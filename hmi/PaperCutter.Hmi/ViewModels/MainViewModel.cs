using System;
using System.Collections.ObjectModel;
using System.IO.Ports;
using System.Linq;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Media;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using PaperCutter.Hmi.Models;
using PaperCutter.Hmi.Protocol;
using PaperCutter.Hmi.Services;

namespace PaperCutter.Hmi.ViewModels
{
    public partial class MainViewModel : ObservableObject
    {
        private readonly ICommunicationService _commService;
        private byte _seq = 1;

        // Connection properties
        [ObservableProperty]
        private ObservableCollection<string> _availablePorts = new();

        [ObservableProperty]
        private string _selectedPort = "COM11";

        [ObservableProperty]
        private string _tcpHost = "127.0.0.1";

        [ObservableProperty]
        private int _tcpPort = 9000;

        [ObservableProperty]
        [NotifyPropertyChangedFor(nameof(UseSerial))]
        private bool _useTcp = false;

        public bool UseSerial => !UseTcp;

        [ObservableProperty]
        [NotifyPropertyChangedFor(nameof(IsDisconnected))]
        [NotifyPropertyChangedFor(nameof(CanLoadJob))]
        [NotifyPropertyChangedFor(nameof(CanStartJob))]
        private bool _isConnected;

        public bool IsDisconnected => !IsConnected;

        [ObservableProperty]
        private string _connectionStatus = "Disconnected";

        [ObservableProperty]
        private ushort _crcErrors;

        [ObservableProperty]
        private ushort _droppedPackets;

        [ObservableProperty]
        private ushort _timeoutErrors;

        // Telemetry properties
        [ObservableProperty]
        private string _stateText = "OFFLINE";

        [ObservableProperty]
        private Brush _stateColor = Brushes.Gray;

        [ObservableProperty]
        private string _faultText = "No Faults";

        [ObservableProperty]
        private bool _isFaultActive;

        [ObservableProperty]
        private double _backGaugePosMm;

        [ObservableProperty]
        private double _bladePosMm;

        [ObservableProperty]
        private bool _feedSensorActive;

        [ObservableProperty]
        private bool _bladeTdcActive = true;

        [ObservableProperty]
        private bool _bladeBdcActive;

        [ObservableProperty]
        private bool _clampSwitchActive;

        [ObservableProperty]
        private bool _clampEngaged;

        [ObservableProperty]
        private int _currentCutIndex;

        [ObservableProperty]
        private double _jobProgress;

        // Job editor
        [ObservableProperty]
        private ObservableCollection<CutItem> _cuts = new();

        [ObservableProperty]
        private double _newCutPositionMm = 50.0;

        [ObservableProperty]
        private ObservableCollection<string> _eventLogs = new();

        public bool CanLoadJob => IsConnected && Cuts.Count > 0;
        public bool CanStartJob => IsConnected && Cuts.Count > 0 && !IsFaultActive;

        public MainViewModel(ICommunicationService commService)
        {
            _commService = commService;
            _commService.ConnectionStateChanged += OnConnectionStateChanged;
            _commService.TelemetryReceived += OnTelemetryReceived;
            _commService.PacketReceived += OnPacketReceived;
            _commService.LogEvent += LogEvent;

            RefreshPorts();

            // Default demo cut recipe
            Cuts.Add(new CutItem(1, 100.0));
            Cuts.Add(new CutItem(2, 80.0));
            Cuts.Add(new CutItem(3, 60.0));
            Cuts.Add(new CutItem(4, 40.0));
            Cuts.Add(new CutItem(5, 20.0));
        }

        public MainViewModel() : this(new CommunicationService())
        {
        }

        [RelayCommand]
        public void RefreshPorts()
        {
            AvailablePorts.Clear();
            var ports = SerialPort.GetPortNames();
            foreach (var p in ports.OrderBy(x => x))
            {
                AvailablePorts.Add(p);
            }
            if (!AvailablePorts.Contains("COM11"))
            {
                AvailablePorts.Add("COM11");
            }
            if (!AvailablePorts.Contains(SelectedPort) && AvailablePorts.Count > 0)
            {
                SelectedPort = AvailablePorts.First();
            }
        }

        [RelayCommand]
        public async Task ConnectAsync()
        {
            ConnectionStatus = "Connecting...";
            bool success = false;
            if (UseTcp)
            {
                success = await _commService.ConnectTcpAsync(TcpHost, TcpPort);
            }
            else
            {
                success = await _commService.ConnectSerialAsync(SelectedPort, 115200);
            }

            if (success)
            {
                // Send HELLO handshake
                _commService.SendPacket(new Packet(_seq++, (byte)Command.Hello, new byte[] { 1, 0, 100 }));
            }
            else
            {
                ConnectionStatus = "Failed to connect";
            }
        }

        [RelayCommand]
        public void Disconnect()
        {
            _commService.Disconnect();
            ConnectionStatus = "Disconnected";
            StateText = "OFFLINE";
            StateColor = Brushes.Gray;
        }

        [RelayCommand]
        public void Estop()
        {
            // Highest priority emergency stop bypass
            _commService.SendEstopImmediate();
            LogEvent(">>> OPERATOR PRESSED HARDWARE E-STOP <<<", true);
        }

        [RelayCommand]
        public void LoadJob()
        {
            if (Cuts.Count == 0) return;
            var positions = Cuts.Select(c => c.PositionUnits).ToArray();
            byte[] packet = PacketEncoder.EncodeLoadJob(_seq++, 101, positions);
            _commService.SendPacket(new Packet { Seq = _seq++, Cmd = (byte)Command.LoadJob, Payload = packet[5..^2] });
            LogEvent($"Loaded job 101 with {Cuts.Count} cuts.", false);
        }

        [RelayCommand]
        public void StartJob()
        {
            byte[] payload = new byte[2];
            System.Buffers.Binary.BinaryPrimitives.WriteUInt16BigEndian(payload, 101);
            _commService.SendPacket(new Packet(_seq++, (byte)Command.Start, payload));
            LogEvent("START command sent.", false);
        }

        [RelayCommand]
        public void PauseResume()
        {
            _commService.SendPacket(new Packet(_seq++, (byte)Command.PauseResume));
            LogEvent("PAUSE / RESUME command sent.", false);
        }

        [RelayCommand]
        public void Abort()
        {
            _commService.SendPacket(new Packet(_seq++, (byte)Command.Abort), highPriority: true);
            LogEvent("ABORT command sent.", false);
        }

        [RelayCommand]
        public void ResetFault()
        {
            _commService.SendPacket(new Packet(_seq++, (byte)Command.ResetFault));
            LogEvent("RESET_FAULT command sent.", false);
        }

        [RelayCommand]
        public void AddCut()
        {
            if (NewCutPositionMm > 0 && Cuts.Count < 60)
            {
                Cuts.Add(new CutItem(Cuts.Count + 1, NewCutPositionMm));
                OnPropertyChanged(nameof(CanLoadJob));
            }
        }

        [RelayCommand]
        public void ClearCuts()
        {
            Cuts.Clear();
            OnPropertyChanged(nameof(CanLoadJob));
        }

        private void OnConnectionStateChanged(bool connected)
        {
            Application.Current?.Dispatcher.Invoke(() =>
            {
                IsConnected = connected;
                ConnectionStatus = connected ? (UseTcp ? $"TCP Connected ({TcpHost}:{TcpPort})" : $"Serial Connected ({SelectedPort})") : "Disconnected";
            });
        }

        private void OnTelemetryReceived(TelemetryData t)
        {
            Application.Current?.Dispatcher.Invoke(() =>
            {
                BackGaugePosMm = t.BackGaugePos / 100.0;
                BladePosMm = t.BladePos / 100.0;
                FeedSensorActive = t.FeedSensor;
                BladeTdcActive = (t.LimitSwitches & LimitSwitchMask.BladeTdc) != 0;
                BladeBdcActive = (t.LimitSwitches & LimitSwitchMask.BladeBdc) != 0;
                ClampSwitchActive = (t.LimitSwitches & LimitSwitchMask.ClampSwitch) != 0;
                ClampEngaged = t.ClampEngaged;
                CurrentCutIndex = t.CutIndex + 1;
                CrcErrors = t.CrcErrors;
                DroppedPackets = t.DroppedPackets;
                TimeoutErrors = t.TimeoutErrors;

                // Update state text & pill color
                switch (t.State)
                {
                    case MachineState.Idle:
                        StateText = "IDLE (READY)";
                        StateColor = Brushes.LimeGreen;
                        break;
                    case MachineState.Feeding:
                        StateText = "FEEDING (MOVING)";
                        StateColor = Brushes.DeepSkyBlue;
                        break;
                    case MachineState.Clamping:
                        StateText = "CLAMPING (HOLDING)";
                        StateColor = Brushes.Orange;
                        break;
                    case MachineState.Cutting:
                        StateText = "CUTTING (CYCLE ACTIVE)";
                        StateColor = Brushes.Crimson;
                        break;
                    case MachineState.ErrorJam:
                        StateText = "ERROR: PAPER JAM";
                        StateColor = Brushes.Red;
                        break;
                    case MachineState.ErrorFault:
                        StateText = $"FAULT: {t.FaultCode}";
                        StateColor = Brushes.Red;
                        break;
                    default:
                        StateText = "UNKNOWN";
                        StateColor = Brushes.Gray;
                        break;
                }

                IsFaultActive = t.State == MachineState.ErrorJam || t.State == MachineState.ErrorFault || t.FaultCode != FaultCode.None;
                FaultText = IsFaultActive ? $"LATCHED FAULT: {t.FaultCode}" : "No Active Faults";

                if (Cuts.Count > 0)
                {
                    JobProgress = Math.Min(100.0, (t.CutIndex / (double)Cuts.Count) * 100.0);
                }
            });
        }

        private void OnPacketReceived(Packet p)
        {
            if (p.Cmd != (byte)Command.Telemetry && p.Cmd != (byte)Command.Heartbeat)
            {
                Application.Current?.Dispatcher.Invoke(() =>
                {
                    string hex = p.Payload.Length > 0 ? Convert.ToHexString(p.Payload) : "EMPTY";
                    LogEvent($"RX: CMD=0x{p.Cmd:X2} SEQ={p.Seq} DATA=[{hex}]", false);
                });
            }
        }

        public void LogEvent(string msg, bool isError)
        {
            Application.Current?.Dispatcher.Invoke(() =>
            {
                string timeStr = DateTime.Now.ToString("HH:mm:ss.fff");
                EventLogs.Insert(0, $"[{timeStr}] {msg}");
                while (EventLogs.Count > 200) EventLogs.RemoveAt(EventLogs.Count - 1);
            });
        }
    }
}
