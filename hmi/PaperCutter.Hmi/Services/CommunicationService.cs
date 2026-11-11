using System;
using System.IO.Ports;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Channels;
using System.Threading.Tasks;
using PaperCutter.Hmi.Protocol;

namespace PaperCutter.Hmi.Services
{
    public interface ICommunicationService : IDisposable
    {
        bool IsConnected { get; }
        event Action<TelemetryData>? TelemetryReceived;
        event Action<Packet>? PacketReceived;
        event Action<string, bool>? LogEvent; // (message, isError)
        event Action<bool>? ConnectionStateChanged;

        Task<bool> ConnectSerialAsync(string portName, int baudRate = 115200);
        Task<bool> ConnectTcpAsync(string host = "127.0.0.1", int port = 9000);
        void Disconnect();

        void SendPacket(Packet packet, bool highPriority = false);
        void SendEstopImmediate();
    }

    public sealed class CommunicationService : ICommunicationService
    {
        private SerialPort? _serialPort;
        private TcpClient? _tcpClient;
        private NetworkStream? _tcpStream;
        private CancellationTokenSource? _cts;

        private readonly PacketParser _parser = new();
        private readonly Channel<byte[]> _normalTxQueue = Channel.CreateUnbounded<byte[]>();
        private readonly Channel<byte[]> _highPriorityTxQueue = Channel.CreateUnbounded<byte[]>();

        private byte _seq = 1;
        private bool _isConnected;

        public bool IsConnected => _isConnected;
        public event Action<TelemetryData>? TelemetryReceived;
        public event Action<Packet>? PacketReceived;
        public event Action<string, bool>? LogEvent;
        public event Action<bool>? ConnectionStateChanged;

        public Task<bool> ConnectSerialAsync(string portName, int baudRate = 115200)
        {
            return Task.Run(() =>
            {
                Disconnect();
                try
                {
                    _serialPort = new SerialPort(portName, baudRate, Parity.None, 8, StopBits.One)
                    {
                        ReadTimeout = 200,
                        WriteTimeout = 200
                    };
                    _serialPort.Open();
                    StartIoThreads();
                    SetConnected(true);
                    LogEvent?.Invoke($"Connected to {portName} at {baudRate} baud", false);
                    return true;
                }
                catch (Exception ex)
                {
                    LogEvent?.Invoke($"Failed to open {portName}: {ex.Message}", true);
                    Disconnect();
                    return false;
                }
            });
        }

        public async Task<bool> ConnectTcpAsync(string host = "127.0.0.1", int port = 9000)
        {
            Disconnect();
            try
            {
                _tcpClient = new TcpClient();
                await _tcpClient.ConnectAsync(host, port);
                _tcpStream = _tcpClient.GetStream();
                StartIoThreads();
                SetConnected(true);
                LogEvent?.Invoke($"Connected to controller over TCP {host}:{port}", false);
                return true;
            }
            catch (Exception ex)
            {
                LogEvent?.Invoke($"Failed to connect to TCP {host}:{port}: {ex.Message}", true);
                Disconnect();
                return false;
            }
        }

        private void SetConnected(bool connected)
        {
            _isConnected = connected;
            ConnectionStateChanged?.Invoke(connected);
        }

        private void StartIoThreads()
        {
            _cts = new CancellationTokenSource();
            Task.Run(() => RxWorkerAsync(_cts.Token));
            Task.Run(() => TxWorkerAsync(_cts.Token));
            Task.Run(() => HeartbeatWorkerAsync(_cts.Token));
        }

        public void Disconnect()
        {
            _cts?.Cancel();
            SetConnected(false);

            try { _serialPort?.Close(); } catch { }
            try { _tcpStream?.Close(); } catch { }
            try { _tcpClient?.Close(); } catch { }

            _serialPort = null;
            _tcpStream = null;
            _tcpClient = null;
            _cts = null;
            _parser.Reset();
        }

        public void SendPacket(Packet packet, bool highPriority = false)
        {
            if (!_isConnected) return;
            byte[] encoded = PacketEncoder.Encode(packet);
            if (highPriority)
            {
                _highPriorityTxQueue.Writer.TryWrite(encoded);
            }
            else
            {
                _normalTxQueue.Writer.TryWrite(encoded);
            }
        }

        public void SendEstopImmediate()
        {
            // Bypasses all queues: immediate write if possible
            byte[] estopWire = PacketEncoder.EncodeEstop(_seq++);
            try
            {
                if (_serialPort != null && _serialPort.IsOpen)
                {
                    _serialPort.Write(estopWire, 0, estopWire.Length);
                }
                else if (_tcpStream != null)
                {
                    _tcpStream.Write(estopWire, 0, estopWire.Length);
                }
            }
            catch { }
            LogEvent?.Invoke("EMERGENCY STOP (ESTOP) packet transmitted immediately!", true);
        }

        private async Task TxWorkerAsync(CancellationToken ct)
        {
            while (!ct.IsCancellationRequested)
            {
                byte[]? data = null;

                // Priority 1: High priority queue (ESTOP, ABORT)
                if (_highPriorityTxQueue.Reader.TryRead(out data) ||
                    await _normalTxQueue.Reader.WaitToReadAsync(ct))
                {
                    if (data == null && _normalTxQueue.Reader.TryRead(out data)) { }

                    if (data != null)
                    {
                        try
                        {
                            if (_serialPort != null && _serialPort.IsOpen)
                            {
                                _serialPort.Write(data, 0, data.Length);
                            }
                            else if (_tcpStream != null)
                            {
                                await _tcpStream.WriteAsync(data, ct);
                            }
                        }
                        catch (Exception ex)
                        {
                            LogEvent?.Invoke($"TX Error: {ex.Message}", true);
                        }
                    }
                }
            }
        }

        private async Task RxWorkerAsync(CancellationToken ct)
        {
            byte[] buffer = new byte[512];
            while (!ct.IsCancellationRequested)
            {
                int bytesRead = 0;
                try
                {
                    if (_serialPort != null && _serialPort.IsOpen && _serialPort.BytesToRead > 0)
                    {
                        bytesRead = _serialPort.Read(buffer, 0, Math.Min(buffer.Length, _serialPort.BytesToRead));
                    }
                    else if (_tcpStream != null)
                    {
                        if (_tcpStream.DataAvailable)
                        {
                            bytesRead = await _tcpStream.ReadAsync(buffer, ct);
                        }
                        else
                        {
                            await Task.Delay(5, ct);
                        }
                    }
                    else
                    {
                        await Task.Delay(10, ct);
                    }
                }
                catch (Exception)
                {
                    if (!ct.IsCancellationRequested)
                    {
                        Disconnect();
                    }
                    break;
                }

                if (bytesRead > 0)
                {
                    for (int i = 0; i < bytesRead; i++)
                    {
                        var pkt = _parser.ProcessByte(buffer[i]);
                        if (pkt != null)
                        {
                            PacketReceived?.Invoke(pkt);
                            if (pkt.Cmd == (byte)Command.Telemetry)
                            {
                                try
                                {
                                    var telem = TelemetryData.Parse(pkt.Payload);
                                    TelemetryReceived?.Invoke(telem);
                                }
                                catch { }
                            }
                        }
                    }
                }
            }
        }

        private async Task HeartbeatWorkerAsync(CancellationToken ct)
        {
            while (!ct.IsCancellationRequested)
            {
                await Task.Delay(100, ct); // 100 ms heartbeat
                if (_isConnected)
                {
                    SendPacket(new Packet(_seq++, (byte)Command.Heartbeat));
                }
            }
        }

        public void Dispose()
        {
            Disconnect();
        }
    }
}
