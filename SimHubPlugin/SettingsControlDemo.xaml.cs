//using SimHub.Plugins.OutputPlugins.Dash.GLCDTemplating;
using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Runtime.InteropServices;
using System.Windows;
using System.Windows.Controls;

using System.Windows.Media.TextFormatting;
using System.Text.Json;
using FMOD;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Json;
using System.IO;
using System.Text;
using System.Web;
using MahApps.Metro.Controls;
using System.Runtime.CompilerServices;
using System.CodeDom.Compiler;
using System.Windows.Forms;
using static System.Net.Mime.MediaTypeNames;
using System.Runtime.InteropServices.ComTypes;
using Microsoft.Win32;
using static System.Windows.Forms.VisualStyles.VisualStyleElement;
using System.Windows.Input;
using System.Windows.Shapes;
using MouseEventArgs = System.Windows.Input.MouseEventArgs;
using SimHub.Plugins.OutputPlugins.GraphicalDash.PSE;
using SimHub.Plugins.Styles;
using System.Windows.Media;
using System.Runtime.Remoting.Messaging;
using SimHub.Plugins.OutputPlugins.GraphicalDash.Behaviors.DoubleText.Imp;
using System.Reflection;
using System.Text.Json.Nodes;
using System.Text.Json.Serialization;
using Newtonsoft.Json;
using System.Threading;
using System.Text.RegularExpressions;
using SimHub.Plugins;
using log4net.Plugin;
//using System.Drawing;

using vJoyInterfaceWrap;
//using vJoy.Wrapper;
using System.Runtime;
using SimHub.Plugins.DataPlugins.ShakeItV3.Settings;
using System.Windows.Media.Effects;
using System.Diagnostics;
using System.Collections;
using System.Linq;

// Win 11 install, see https://github.com/jshafer817/vJoy/releases
//using vJoy.Wrapper;



namespace User.PluginSdkDemo
{


    /// <summary>
    /// Logique d'interaction pour SettingsControlDemo.xaml
    /// </summary>
    public partial class SettingsControlDemo : System.Windows.Controls.UserControl
    {


        // payload revisiom
        //public uint pedalConfigPayload_version = 110;
        //public uint pedalConfigPayload_type = 100;
        //public uint pedalActionPayload_type = 110;

        public uint indexOfSelectedPedal_u = 1;
        public uint profile_select = 0;
        public DIY_FFB_Pedal Plugin { get; }


        public DAP_config_st[] dap_config_st = new DAP_config_st[3];
        private string stringValue;


        public bool[] waiting_for_pedal_config = new bool[3];
        public System.Windows.Forms.Timer[] pedal_serial_read_timer = new System.Windows.Forms.Timer[3];
        public System.Windows.Forms.Timer connect_timer;
        //public System.Timers.Timer[] pedal_serial_read_timer = new System.Timers.Timer[3];
        int printCtr = 0;

        public double[] Force_curve_Y = new double[100];
        public bool debug_flag = false;

        //public VirtualJoystick joystick;
        internal vJoyInterfaceWrap.vJoy joystick;


        public bool[] dumpPedalToResponseFile = new bool[3];

        private SolidColorBrush defaultcolor;
        private SolidColorBrush lightcolor;



        // read config from JSON on startup
        //ReadStructFromJson();


        // read JSON config from JSON file
        //private void ReadStructFromJson()
        //{



        //    try
        //    {
        //        // https://learn.microsoft.com/en-us/dotnet/standard/serialization/system-text-json/how-to?pivots=dotnet-8-0
        //        // https://www.educative.io/answers/how-to-read-a-json-file-in-c-sharp

        //        string currentDirectory = Directory.GetCurrentDirectory();
        //        string dirName = currentDirectory + "\\PluginsData\\Common";
        //        //string jsonFileName = ComboBox_JsonFileSelected.Text;
        //        string jsonFileName = ((ComboBoxItem)ComboBox_JsonFileSelected.SelectedItem).Content.ToString();
        //        string fileName = dirName + "\\" + jsonFileName + ".json";

        //        string text = System.IO.File.ReadAllText(fileName);

        //        DataContractJsonSerializer deserializer = new DataContractJsonSerializer(typeof(DAP_config_st));
        //        var ms = new MemoryStream(Encoding.UTF8.GetBytes(text));
        //        dap_config_st[indexOfSelectedPedal_u] = (DAP_config_st)deserializer.ReadObject(ms);
        //        //TextBox_debugOutput.Text = "Config loaded!";
        //        //TextBox_debugOutput.Text += ComboBox_JsonFileSelected.Text;
        //        //TextBox_debugOutput.Text += "    ";
        //        //TextBox_debugOutput.Text += ComboBox_JsonFileSelected.SelectedIndex;

        //        updateTheGuiFromConfig();

        //    }
        //    catch (Exception caughtEx)
        //    {

        //        string errorMessage = caughtEx.Message;
        //        TextBox_debugOutput.Text = errorMessage;
        //    }


        //}
        private void vjoy_axis_initialize()
        {
            //center all axis/hats reader
            joystick.SetAxis(16384, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_X);
            joystick.SetAxis(16384, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_Y);
            joystick.SetAxis(16384, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_Z);
            joystick.SetAxis(16384, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_RX);
            joystick.SetAxis(16384, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_RY);
            joystick.SetAxis(16384, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_RZ);
            //joystick.SetJoystickHat(0, Hats.Hat);
            //joystick.SetJoystickHat(0, Hats.HatExt1);
            //joystick.SetJoystickHat(0, Hats.HatExt2);
            //joystick.SetJoystickHat(0, Hats.HatExt3);

        }
        private void DrawGridLines()
        {
            // Specify the number of rows and columns for the grid
            int rowCount = 5;
            int columnCount = 5;

            // Calculate the width and height of each cell
            double cellWidth = canvas.Width / columnCount;
            double cellHeight = canvas.Height / rowCount;

            // Draw horizontal gridlines
            for (int i = 1; i < rowCount; i++)
            {
                Line line = new Line
                {
                    X1 = 0,
                    Y1 = i * cellHeight,
                    X2 = canvas.Width,
                    Y2 = i * cellHeight,
                    //Stroke = Brush.Black,
                    Stroke = System.Windows.Media.Brushes.LightSteelBlue,
                    StrokeThickness = 1,
                    Opacity = 0.1

                };
                canvas.Children.Add(line);
            }

            // Draw vertical gridlines
            for (int i = 1; i < columnCount; i++)
            {
                Line line = new Line
                {
                    X1 = i * cellWidth,
                    Y1 = 0,
                    X2 = i * cellWidth,
                    Y2 = canvas.Height,
                    //Stroke = Brushes.Black,
                    Stroke = System.Windows.Media.Brushes.LightSteelBlue,
                    StrokeThickness = 1,
                    Opacity = 0.1
                };
                canvas.Children.Add(line);
            }
        }

        private void InitReadStructFromJson()
        {



            try
            {
                // https://learn.microsoft.com/en-us/dotnet/standard/serialization/system-text-json/how-to?pivots=dotnet-8-0
                // https://www.educative.io/answers/how-to-read-a-json-file-in-c-sharp
                string jsonFileName = "NA";

                string currentDirectory = Directory.GetCurrentDirectory();
                string dirName = currentDirectory + "\\PluginsData\\Common";
                //string jsonFileName = ComboBox_JsonFileSelected.Text;
                if (indexOfSelectedPedal_u == 0)
                {
                    jsonFileName = ("DiyPedalConfig_Clutch_Default");
                }
                else if (indexOfSelectedPedal_u == 1)
                {
                    jsonFileName = ("DiyPedalConfig_Brake_Default");
                }
                else if (indexOfSelectedPedal_u == 2)
                {
                    jsonFileName = ("DiyPedalConfig_Accelerator_Default");
                }

                string fileName = dirName + "\\" + jsonFileName + ".json";
                string text = System.IO.File.ReadAllText(fileName);



                DataContractJsonSerializer deserializer = new DataContractJsonSerializer(typeof(DAP_config_st));
                var ms = new MemoryStream(Encoding.UTF8.GetBytes(text));
                dap_config_st[indexOfSelectedPedal_u] = (DAP_config_st)deserializer.ReadObject(ms);
                TextBox_debugOutput.Text = "Config loaded!" + jsonFileName;
                //TextBox_debugOutput.Text += ComboBox_JsonFileSelected.Text;
                //TextBox_debugOutput.Text += "    ";
                //TextBox_debugOutput.Text += ComboBox_JsonFileSelected.SelectedIndex;

                updateTheGuiFromConfig();

            }
            catch (Exception caughtEx)
            {

                string errorMessage = caughtEx.Message;
                TextBox_debugOutput.Text = errorMessage;
            }


        }

        private void UpdateSerialPortList_click()
        {

            var SerialPortSelectionArray = new List<SerialPortChoice>();
            string[] comPorts = SerialPort.GetPortNames();
            if (comPorts.Length > 0)
            {

                foreach (string portName in comPorts)
                {
                    SerialPortSelectionArray.Add(new SerialPortChoice(portName, portName));
                }
            }
            else
            {
                SerialPortSelectionArray.Add(new SerialPortChoice("NA", "NA"));
            }

            SerialPortSelection.DataContext = SerialPortSelectionArray;
        }

        private bool isDragging = false;
        private Point offset;

        public void DAP_config_set_default(uint pedalIdx)
        {
            dumpPedalToResponseFile[pedalIdx] = false;
            dap_config_st[pedalIdx].payloadHeader_.payloadType = (byte)Constants.pedalConfigPayload_type;
            dap_config_st[pedalIdx].payloadHeader_.version = (byte)Constants.pedalConfigPayload_version;
            dap_config_st[pedalIdx].payloadPedalConfig_.pedalStartPosition = 35;
            dap_config_st[pedalIdx].payloadPedalConfig_.pedalEndPosition = 80;
            dap_config_st[pedalIdx].payloadPedalConfig_.maxForce = 90;
            dap_config_st[pedalIdx].payloadPedalConfig_.relativeForce_p000 = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.relativeForce_p020 = 20;
            dap_config_st[pedalIdx].payloadPedalConfig_.relativeForce_p040 = 40;
            dap_config_st[pedalIdx].payloadPedalConfig_.relativeForce_p060 = 60;
            dap_config_st[pedalIdx].payloadPedalConfig_.relativeForce_p080 = 80;
            dap_config_st[pedalIdx].payloadPedalConfig_.relativeForce_p100 = 100;
            dap_config_st[pedalIdx].payloadPedalConfig_.dampingPress = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.dampingPull = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.absFrequency = 5;
            dap_config_st[pedalIdx].payloadPedalConfig_.absAmplitude = 20;
            dap_config_st[pedalIdx].payloadPedalConfig_.absPattern = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.absForceOrTarvelBit = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.lengthPedal_AC = 150;
            dap_config_st[pedalIdx].payloadPedalConfig_.horPos_AB = 215;
            dap_config_st[pedalIdx].payloadPedalConfig_.verPos_AB = 80;
            dap_config_st[pedalIdx].payloadPedalConfig_.lengthPedal_CB = 200;
            dap_config_st[pedalIdx].payloadPedalConfig_.Simulate_ABS_trigger = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.Simulate_ABS_value = 80;
            dap_config_st[pedalIdx].payloadPedalConfig_.RPM_max_freq = 40;
            dap_config_st[pedalIdx].payloadPedalConfig_.RPM_min_freq = 10;
            dap_config_st[pedalIdx].payloadPedalConfig_.RPM_AMP = 30;
            dap_config_st[pedalIdx].payloadPedalConfig_.BP_trigger_value = 50;
            dap_config_st[pedalIdx].payloadPedalConfig_.BP_amp = 1;
            dap_config_st[pedalIdx].payloadPedalConfig_.BP_freq = 15;
            dap_config_st[pedalIdx].payloadPedalConfig_.BP_trigger = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.G_multi = 50;
            dap_config_st[pedalIdx].payloadPedalConfig_.G_window = 60;
            dap_config_st[pedalIdx].payloadPedalConfig_.maxGameOutput = 100;
            dap_config_st[pedalIdx].payloadPedalConfig_.kf_modelNoise = 128;
            dap_config_st[pedalIdx].payloadPedalConfig_.cubic_spline_param_a_0 = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.cubic_spline_param_a_1 = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.cubic_spline_param_a_2 = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.cubic_spline_param_a_3 = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.cubic_spline_param_a_4 = 0;

            dap_config_st[pedalIdx].payloadPedalConfig_.cubic_spline_param_b_0 = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.cubic_spline_param_b_1 = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.cubic_spline_param_b_2 = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.cubic_spline_param_b_3 = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.cubic_spline_param_b_4 = 0;

            dap_config_st[pedalIdx].payloadPedalConfig_.PID_p_gain = 0.3f;
            dap_config_st[pedalIdx].payloadPedalConfig_.PID_i_gain = 50.0f;
            dap_config_st[pedalIdx].payloadPedalConfig_.PID_d_gain = 0.0f;
            dap_config_st[pedalIdx].payloadPedalConfig_.PID_velocity_feedforward_gain = 0.0f;

            dap_config_st[pedalIdx].payloadPedalConfig_.MPC_0th_order_gain = 1.0f;

            dap_config_st[pedalIdx].payloadPedalConfig_.control_strategy_b = 2;

            dap_config_st[pedalIdx].payloadPedalConfig_.loadcell_rating = 150;

            dap_config_st[pedalIdx].payloadPedalConfig_.travelAsJoystickOutput_u8 = 0;

            dap_config_st[pedalIdx].payloadPedalConfig_.invertLoadcellReading_u8 = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.invertMotorDirection_u8 = 0;

            dap_config_st[pedalIdx].payloadPedalConfig_.spindlePitch_mmPerRev_u8 = 5;
        }


        public SettingsControlDemo()
        {


            for (uint pedalIdx = 0; pedalIdx < 3; pedalIdx++)
            {


                DAP_config_set_default(pedalIdx);
                InitializeComponent();


            }
            // debug mode invisiable
            text_debug_flag.Visibility = Visibility.Hidden;
            text_serial.Visibility = Visibility.Hidden;
            TextBox_serialMonitor.Visibility = System.Windows.Visibility.Hidden;
            InvertLoadcellReading_check.Visibility = Visibility.Hidden;
            InvertMotorDir_check.Visibility = Visibility.Hidden;
            textBox_debug_Flag_0.Visibility = Visibility.Hidden;
            Border_serial_monitor.Visibility=Visibility.Hidden;
            debug_border.Visibility=Visibility.Hidden;
            debug_label_text.Visibility=Visibility.Hidden;
            //btn_serial.Visibility = System.Windows.Visibility.Hidden;
            button_pedal_position_reset.Visibility = System.Windows.Visibility.Hidden;
            button_pedal_restart.Visibility = System.Windows.Visibility.Hidden;
            btn_system_id.Visibility = System.Windows.Visibility.Hidden;
            btn_reset_default.Visibility = System.Windows.Visibility.Hidden;
            dump_pedal_response_to_file.Visibility = System.Windows.Visibility.Hidden;
            //setting drawing color with Simhub theme workaround
            SolidColorBrush buttonBackground_ = btn_update.Background as SolidColorBrush;

            Color color = Color.FromArgb(150, buttonBackground_.Color.R, buttonBackground_.Color.G, buttonBackground_.Color.B);
            Color color_2 = Color.FromArgb(200, buttonBackground_.Color.R, buttonBackground_.Color.G, buttonBackground_.Color.B);
            Color color_3 = Color.FromArgb(255, buttonBackground_.Color.R, buttonBackground_.Color.G, buttonBackground_.Color.B);
            SolidColorBrush Line_fill = new SolidColorBrush(color_2);
            //SolidColorBrush rect_fill = new SolidColorBrush(color);
            defaultcolor = new SolidColorBrush(color);
            lightcolor = new SolidColorBrush(color_3);

            text_min_force.Foreground = Line_fill;
            text_max_force.Foreground = Line_fill;
            text_max_pos.Foreground = Line_fill;
            text_min_pos.Foreground = Line_fill;
            text_position.Foreground = Line_fill;
            rect0.Fill = defaultcolor;
            rect1.Fill = defaultcolor;
            rect2.Fill = defaultcolor;
            rect3.Fill = defaultcolor;
            rect4.Fill = defaultcolor;
            rect5.Fill = defaultcolor;
            rect6.Fill = defaultcolor;
            rect7.Fill = defaultcolor;
            rect8.Fill = defaultcolor;
            rect9.Fill = defaultcolor;
            Line_V_force.Stroke = Line_fill;
            Line_H_pos.Stroke = Line_fill;
            //Polyline_BrakeForceCurve.Stroke = new SolidColorBrush(Line_fill);
            Polyline_BrakeForceCurve.Stroke = Line_fill;
            text_damping_text.Foreground = Line_fill;
            Line_H_damping.Stroke = Line_fill;
            text_damping.Foreground = Line_fill;
            rect_damping.Fill = defaultcolor;

            text_Pgain_text.Foreground = Line_fill;
            Line_H_Pgain.Stroke = Line_fill;
            text_Pgain.Foreground = Line_fill;
            rect_Pgain.Fill = defaultcolor;

            text_Igain_text.Foreground = Line_fill;
            Line_H_Igain.Stroke = Line_fill;
            text_Igain.Foreground = Line_fill;
            rect_Igain.Fill = defaultcolor;

            text_Dgain_text.Foreground = Line_fill;
            Line_H_Dgain.Stroke = Line_fill;
            text_Dgain.Foreground = Line_fill;
            rect_Dgain.Fill = defaultcolor;

            text_VFgain_text.Foreground = Line_fill;
            Line_H_VFgain.Stroke = Line_fill;
            text_VFgain.Foreground = Line_fill;
            rect_VFgain.Fill = defaultcolor;

            Line_H_ABS.Stroke = Line_fill;
            text_ABS.Foreground = Line_fill;
            rect_ABS.Fill = defaultcolor;
            text_ABS_text.Foreground = Line_fill;
            Line_H_ABS_freq.Stroke = Line_fill;
            text_ABS_freq.Foreground = Line_fill;
            rect_ABS_freq.Fill = defaultcolor;
            text_ABS_freq_text.Foreground = Line_fill;
            Line_H_max_game.Stroke = Line_fill;
            text_max_game.Foreground = Line_fill;
            text_max_game_text.Foreground = Line_fill;
            rect_max_game.Fill = defaultcolor;

            Line_H_KF.Stroke = Line_fill;
            text_KF.Foreground = Line_fill;
            rect_KF.Fill = defaultcolor;
            text_KF_text.Foreground = Line_fill;
            Line_H_LC_rating.Stroke = Line_fill;
            text_LC_rating.Foreground = Line_fill;
            text_LC_rating_text.Foreground = Line_fill;
            rect_LC_rating.Fill = defaultcolor;

            text_RPM_freq_min.Foreground = Line_fill;
            text_RPM_freq_max.Foreground = Line_fill;
            text_RPM_AMP.Foreground = Line_fill;
            Line_H_RPM_AMP.Stroke = Line_fill;
            rect_RPM_AMP.Fill = defaultcolor;
            text_RPM_AMP_text.Foreground =

            Line_H_RPM_freq.Stroke = Line_fill;
            rect_RPM_max.Fill = Line_fill;
            rect_RPM_min.Fill = defaultcolor;
            text_RPM_freq_text.Foreground = Line_fill;

            text_bite_amp.Foreground = Line_fill;
            text_bite_freq.Foreground =
            rect_bite_amp.Fill = defaultcolor;
            rect_bite_freq.Fill = defaultcolor;
            text_bite_amp_text.Foreground = Line_fill;
            text_bite_freq_text.Foreground = Line_fill;
            Line_H_bite_amp.Stroke = Line_fill;
            Line_H_bite_freq.Stroke = Line_fill;

            Line_G_force_multi.Stroke = Line_fill;
            text_G_force_multi_text.Foreground = Line_fill;
            text_G_multi.Foreground = Line_fill;
            rect_G_force_multi.Fill = defaultcolor;

            Line_G_force_window.Stroke = Line_fill;
            text_G_force_window_text.Foreground = Line_fill;
            text_G_window.Foreground = Line_fill;
            rect_G_force_window.Fill = defaultcolor;

            text_MPC_0th_order_gain.Foreground = Line_fill;
            text_MPC_0th_order_gain_text.Foreground = Line_fill;
            Line_H_MPC_0th_order_gain.Stroke = Line_fill;
            rect_MPC_0th_order_gain.Fill = defaultcolor;

            Line_H_HeaderTab.Stroke = Line_fill;

            //text_MPC_1st_order_gain.Foreground = Line_fill;
            //text_MPC_1st_order_gain_text.Foreground = Line_fill;
            //Line_H_MPC_1st_order_gain.Stroke = Line_fill;
            //rect_MPC_1st_order_gain.Fill = defaultcolor;






            // Call this method to generate gridlines on the Canvas
            DrawGridLines();









        }



        public byte[] getBytesPayload(payloadPedalConfig aux)
        {
            int length = Marshal.SizeOf(aux);
            IntPtr ptr = Marshal.AllocHGlobal(length);
            byte[] myBuffer = new byte[length];

            Marshal.StructureToPtr(aux, ptr, true);
            Marshal.Copy(ptr, myBuffer, 0, length);
            Marshal.FreeHGlobal(ptr);

            return myBuffer;
        }


        public byte[] getBytes(DAP_config_st aux)
        {
            int length = Marshal.SizeOf(aux);
            IntPtr ptr = Marshal.AllocHGlobal(length);
            byte[] myBuffer = new byte[length];

            Marshal.StructureToPtr(aux, ptr, true);
            Marshal.Copy(ptr, myBuffer, 0, length);
            Marshal.FreeHGlobal(ptr);

            return myBuffer;
        }


        //public byte[] getBytes_Action(DAP_action_st aux)
        //{
        //    int length = Marshal.SizeOf(aux);
        //    IntPtr ptr = Marshal.AllocHGlobal(length);
        //    byte[] myBuffer = new byte[length];

        //    Marshal.StructureToPtr(aux, ptr, true);
        //    Marshal.Copy(ptr, myBuffer, 0, length);
        //    Marshal.FreeHGlobal(ptr);

        //    return myBuffer;
        //}


        public DAP_config_st getConfigFromBytes(byte[] myBuffer)
        {
            DAP_config_st aux;

            // see https://stackoverflow.com/questions/31045358/how-do-i-copy-bytes-into-a-struct-variable-in-c
            int size = Marshal.SizeOf(typeof(DAP_config_st));
            IntPtr ptr = Marshal.AllocHGlobal(size);

            Marshal.Copy(myBuffer, 0, ptr, size);

            aux = (DAP_config_st)Marshal.PtrToStructure(ptr, typeof(DAP_config_st));
            Marshal.FreeHGlobal(ptr);

            return aux;
        }


        public DAP_state_basic_st getStateFromBytes(byte[] myBuffer)
        {
            DAP_state_basic_st aux;

            // see https://stackoverflow.com/questions/31045358/how-do-i-copy-bytes-into-a-struct-variable-in-c
            int size = Marshal.SizeOf(typeof(DAP_state_basic_st));
            IntPtr ptr = Marshal.AllocHGlobal(size);

            Marshal.Copy(myBuffer, 0, ptr, size);

            aux = (DAP_state_basic_st)Marshal.PtrToStructure(ptr, typeof(DAP_state_basic_st));
            Marshal.FreeHGlobal(ptr);

            return aux;
        }

        public DAP_state_extended_st getStateExtFromBytes(byte[] myBuffer)
        {
            DAP_state_extended_st aux;

            // see https://stackoverflow.com/questions/31045358/how-do-i-copy-bytes-into-a-struct-variable-in-c
            int size = Marshal.SizeOf(typeof(DAP_state_extended_st));
            IntPtr ptr = Marshal.AllocHGlobal(size);

            Marshal.Copy(myBuffer, 0, ptr, size);

            aux = (DAP_state_extended_st)Marshal.PtrToStructure(ptr, typeof(DAP_state_extended_st));
            Marshal.FreeHGlobal(ptr);

            return aux;
        }


        //unsafe private UInt16 checksumCalc(byte* data, int length)
        //{

        //    UInt16 curr_crc = 0x0000;
        //    byte sum1 = (byte)curr_crc;
        //    byte sum2 = (byte)(curr_crc >> 8);
        //    int index;
        //    for (index = 0; index < length; index = index + 1)
        //    {
        //        int v = (sum1 + (*data));
        //        sum1 = (byte)v;
        //        sum1 = (byte)(v % 255);

        //        int w = (sum1 + sum2) % 255;
        //        sum2 = (byte)w;

        //        data++;// = data++;
        //    }

        //    int x = (sum2 << 8) | sum1;
        //    return (UInt16)x;
        //}


        public SettingsControlDemo(DIY_FFB_Pedal plugin) : this()
        {
            this.Plugin = plugin;
            plugin.testValue = 1;
            plugin.wpfHandle = this;


            UpdateSerialPortList_click();
            //closeSerialAndStopReadCallback(1);

             

            // check if Json config files are present, otherwise create new ones
            //for (int jsonIndex = 0; jsonIndex < ComboBox_JsonFileSelected.Items.Count; jsonIndex++)
            //{

            //    ComboBox_JsonFileSelected.SelectedIndex = jsonIndex;

            //    // which config file is seleced
            //    string currentDirectory = Directory.GetCurrentDirectory();
            //    string dirName = currentDirectory + "\\PluginsData\\Common";
            //    //string jsonFileName = ComboBox_JsonFileSelected(ComboBox_JsonFileSelected.Items[jsonIndex]).Text;
            //    string jsonFileName = ((ComboBoxItem)ComboBox_JsonFileSelected.SelectedItem).Content.ToString();
            //    string fileName = dirName + "\\" + jsonFileName + ".json";


            //    // Check if file already exists, otherwise create    
            //    if (!File.Exists(fileName))
            //    {
            //        // create default config
            //        // https://stackoverflow.com/questions/3275863/does-net-4-have-a-built-in-json-serializer-deserializer
            //        // https://learn.microsoft.com/en-us/dotnet/framework/wcf/feature-details/how-to-serialize-and-deserialize-json-data?redirectedfrom=MSDN
            //        var stream1 = new MemoryStream();
            //        var ser = new DataContractJsonSerializer(typeof(DAP_config_st));
            //        ser.WriteObject(stream1, Plugin.dap_config_initial_st);

            //        stream1.Position = 0;
            //        StreamReader sr = new StreamReader(stream1);
            //        string jsonString = sr.ReadToEnd();

            //        System.IO.File.WriteAllText(fileName, jsonString);
            //    }
            //}

            string currentDirectory = Directory.GetCurrentDirectory();
            string dirName = currentDirectory + "\\PluginsData\\Common";
            //string jsonFileName = ComboBox_JsonFileSelected(ComboBox_JsonFileSelected.Items[jsonIndex]).Text;
            string jsonFileNameA = "DiyPedalConfig_Accelerator_Default";
            string jsonFileNameB = "DiyPedalConfig_Brake_Default";
            string jsonFileNameC = "DiyPedalConfig_Clutch_Default";
            string fileNameA = dirName + "\\" + jsonFileNameA + ".json";
            string fileNameB = dirName + "\\" + jsonFileNameB + ".json";
            string fileNameC = dirName + "\\" + jsonFileNameC + ".json";

            if (!File.Exists(fileNameA))
            {
                // create default config
                // https://stackoverflow.com/questions/3275863/does-net-4-have-a-built-in-json-serializer-deserializer
                // https://learn.microsoft.com/en-us/dotnet/framework/wcf/feature-details/how-to-serialize-and-deserialize-json-data?redirectedfrom=MSDN
                var stream1 = new MemoryStream();
                var ser = new DataContractJsonSerializer(typeof(DAP_config_st));
                ser.WriteObject(stream1, Plugin.dap_config_initial_st);

                stream1.Position = 0;
                StreamReader sr = new StreamReader(stream1);
                string jsonString = sr.ReadToEnd();

                System.IO.File.WriteAllText(fileNameA, jsonString);
            }

            if (!File.Exists(fileNameB))
            {
                // create default config
                // https://stackoverflow.com/questions/3275863/does-net-4-have-a-built-in-json-serializer-deserializer
                // https://learn.microsoft.com/en-us/dotnet/framework/wcf/feature-details/how-to-serialize-and-deserialize-json-data?redirectedfrom=MSDN
                var stream1 = new MemoryStream();
                var ser = new DataContractJsonSerializer(typeof(DAP_config_st));
                ser.WriteObject(stream1, Plugin.dap_config_initial_st);

                stream1.Position = 0;
                StreamReader sr = new StreamReader(stream1);
                string jsonString = sr.ReadToEnd();

                System.IO.File.WriteAllText(fileNameB, jsonString);
            }
            if (!File.Exists(fileNameC))
            {
                // create default config
                // https://stackoverflow.com/questions/3275863/does-net-4-have-a-built-in-json-serializer-deserializer
                // https://learn.microsoft.com/en-us/dotnet/framework/wcf/feature-details/how-to-serialize-and-deserialize-json-data?redirectedfrom=MSDN
                var stream1 = new MemoryStream();
                var ser = new DataContractJsonSerializer(typeof(DAP_config_st));
                ser.WriteObject(stream1, Plugin.dap_config_initial_st);

                stream1.Position = 0;
                StreamReader sr = new StreamReader(stream1);
                string jsonString = sr.ReadToEnd();

                System.IO.File.WriteAllText(fileNameC, jsonString);
            }
            InitReadStructFromJson();
            for (uint pedalIndex = 0; pedalIndex < 3; pedalIndex++)
            {
                //indexOfSelectedPedal_u = pedalIndex;
                //ComboBox_JsonFileSelected.SelectedIndex = Plugin.Settings.selectedJsonFileNames[indexOfSelectedPedal_u];
                //ComboBox_JsonFileSelected.SelectedIndex = Plugin.Settings.selectedJsonIndexLast[indexOfSelectedPedal_u];
                InitReadStructFromJson();
                /*
                if (plugin.Settings.connect_status[pedalIndex] == 1)
                {
                    if (plugin.Settings.reading_config == 1)
                    {
                        if (plugin._serialPort[pedalIndex].IsOpen)
                        {
                            Reading_config_auto(pedalIndex);
                        }
                        else
                        {
                            plugin.Settings.connect_status[pedalIndex] = 0;
                        }
                        
                    }

                }
                */


                /*
                if (plugin.PortExists(plugin._serialPort[pedalIndex].PortName))
                {
                    if (plugin.Settings.connect_status[pedalIndex] == 1)
                    {
                        if (plugin.Settings.reading_config == 1)
                        {
                            Reading_config_auto(pedalIndex);
                        }

                    }
                    
                }
                else
                {
                    plugin.Settings.connect_status[pedalIndex] = 0;
                }
                */


                updateTheGuiFromConfig();
            }

            if (plugin.Settings.reading_config == 1)
            {
                checkbox_pedal_read.IsChecked = true;

            }
            else
            {
                checkbox_pedal_read.IsChecked = false;
            }
            indexOfSelectedPedal_u = plugin.Settings.table_selected;
            MyTab.SelectedIndex = (int)indexOfSelectedPedal_u;

            //reconnect to com port
            if (plugin.Settings.auto_connect_flag == 1)
            {
                checkbox_auto_connect.IsChecked = true;
            }
            else
            {
                checkbox_auto_connect.IsChecked = false;
            }

            //auto connection with timmer
            if (connect_timer != null)
            {
                connect_timer.Dispose();
                connect_timer.Stop();
            }

            connect_timer = new System.Windows.Forms.Timer();
            connect_timer.Tick += new EventHandler(connection_timmer_tick);
            connect_timer.Interval = 5000; // in miliseconds try connect every 5s
            connect_timer.Start();
            System.Threading.Thread.Sleep(50);

            /*
            // autoconnect serial
            for (uint pedalIdx = 0; pedalIdx < 3; pedalIdx++)
            {
                if (Plugin.connectSerialPort[pedalIdx] == true)
                {
                    if (Plugin.PortExists(Plugin._serialPort[pedalIdx].PortName))
                    {
                        if (Plugin._serialPort[pedalIdx].IsOpen == false)
                        {
                            if (Plugin.Settings.connect_status[pedalIdx] == 1)
                            {
                                openSerialAndAddReadCallback(pedalIdx);
                                Reading_config_auto(pedalIdx);
                            }

                        }
                    }
                    else
                    {
                        Plugin.connectSerialPort[pedalIdx] = false;
                        Plugin.Settings.connect_status[pedalIdx] = 0;
                    }

                }
            }
            */


            //vjoy initialized
            if (Plugin.Settings.vjoy_output_flag == 1)
            {
                Vjoy_out_check.IsChecked = true;
                uint vJoystickId = Plugin.Settings.vjoy_order;
                //joystick = new VirtualJoystick(Plugin.Settings.vjoy_order);
                joystick = new vJoyInterfaceWrap.vJoy();

                joystick.AcquireVJD(vJoystickId);
                //joystick.Aquire();
                vjoy_axis_initialize();
            }
            else
            {
                Vjoy_out_check.IsChecked = false;
            }


        }




        public void updateTheGuiFromConfig()
        {
            // update the sliders


            info_label.Content = "State:\nDAP Version:";
            string info_text;
            if (Plugin._serialPort[indexOfSelectedPedal_u].IsOpen)
            {
                info_text = "Connected";
            }
            else
            {
                if (Plugin.Settings.auto_connect_flag == 1)
                {
                    info_text = "Connecting..";
                }
                else
                {
                    info_text = "Waiting...";
                }

            }
            info_text += "\n" + Constants.pedalConfigPayload_version;
            /*if ((bool)TestAbs_check.IsChecked)
            {
                info_text += "\nABS/TC Testing";
            }*/
            info_label_2.Content = info_text;


            int debugFlagValue_0 = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.debug_flags_0;
            textBox_debug_Flag_0.Text = debugFlagValue_0.ToString();



            Update_BrakeForceCurve();
            //Simulated ABS trigger
            if (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.Simulate_ABS_trigger == 1)
            {
                Simulate_ABS_check.IsChecked = true;
            }
            else
            {
                Simulate_ABS_check.IsChecked = false;
            }


            if (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.control_strategy_b == 0)
            {
                ControlStrategy_Sel_1.IsChecked = true;
            }
            if (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.control_strategy_b == 1)
            {
                ControlStrategy_Sel_2.IsChecked = true;
            }
            if (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.control_strategy_b == 2)
            {
                ControlStrategy_Sel_3.IsChecked = true;
            }



            //set control point position
            text_point_pos.Visibility = Visibility.Hidden;
            double control_rect_value_max = 100;
            double dyy = canvas.Height / control_rect_value_max;
            Canvas.SetTop(rect0, canvas.Height - dyy * dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.relativeForce_p000 - rect0.Height / 2);
            Canvas.SetLeft(rect0, 0 * canvas.Width / 5 - rect0.Width / 2);
            Canvas.SetTop(rect1, canvas.Height - dyy * dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.relativeForce_p020 - rect0.Height / 2);
            Canvas.SetLeft(rect1, 1 * canvas.Width / 5 - rect1.Width / 2);
            Canvas.SetTop(rect2, canvas.Height - dyy * dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.relativeForce_p040 - rect0.Height / 2);
            Canvas.SetLeft(rect2, 2 * canvas.Width / 5 - rect2.Width / 2);
            Canvas.SetTop(rect3, canvas.Height - dyy * dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.relativeForce_p060 - rect0.Height / 2);
            Canvas.SetLeft(rect3, 3 * canvas.Width / 5 - rect3.Width / 2);
            Canvas.SetTop(rect4, canvas.Height - dyy * dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.relativeForce_p080 - rect0.Height / 2);
            Canvas.SetLeft(rect4, 4 * canvas.Width / 5 - rect4.Width / 2);
            Canvas.SetTop(rect5, canvas.Height - dyy * dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.relativeForce_p100 - rect0.Height / 2);
            Canvas.SetLeft(rect5, 5 * canvas.Width / 5 - rect5.Width / 2);
            if (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.debug_flags_0 != 32)
            {
                rect_State.Visibility = Visibility.Visible;
                text_state.Visibility = Visibility.Visible;
            }
            else
            {
                rect_State.Visibility = Visibility.Hidden;
                text_state.Visibility = Visibility.Hidden;
            }
            Canvas.SetTop(rect_State, canvas.Height - rect_State.Height / 2);
            Canvas.SetLeft(rect_State, -rect_State.Width / 2);
            Canvas.SetLeft(text_state, Canvas.GetLeft(rect_State) /*+ rect_State.Width*/);
            Canvas.SetTop(text_state, Canvas.GetTop(rect_State) - rect_State.Height);
            text_state.Text = "0%";
            //set for ABS slider
            Canvas.SetTop(rect_SABS_Control, (control_rect_value_max - dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.Simulate_ABS_value) * dyy - rect_SABS_Control.Height / 2);
            Canvas.SetLeft(rect_SABS_Control, 0);
            Canvas.SetTop(rect_SABS, 0);
            Canvas.SetLeft(rect_SABS, 0);
            rect_SABS.Height = canvas.Height - dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.Simulate_ABS_value * dyy;
            Canvas.SetTop(text_SABS, Canvas.GetTop(rect_SABS_Control) - text_SABS.Height - rect_SABS_Control.Height);
            Canvas.SetLeft(text_SABS, canvas.Width - text_SABS.Width);
            text_SABS.Text = "ABS trigger value: " + dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.Simulate_ABS_value + "%";
            if (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.Simulate_ABS_trigger == 1)
            {
                rect_SABS.Visibility = Visibility.Visible;
                rect_SABS_Control.Visibility = Visibility.Visible;
                text_SABS.Visibility = Visibility.Visible;
            }
            else
            {
                rect_SABS.Visibility = Visibility.Hidden;
                rect_SABS_Control.Visibility = Visibility.Hidden;
                text_SABS.Visibility = Visibility.Hidden;
            }
            //set for travel slider;
            double dx = (canvas_horz_slider.Width - 10) / 100;
            Canvas.SetTop(rect6, 15);
            //TextBox_debugOutput.Text= Convert.ToString(dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalStartPosition);
            Canvas.SetLeft(rect6, rect6.Width / 2 + dx * dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalStartPosition);
            Canvas.SetLeft(text_min_pos, Canvas.GetLeft(rect6) - text_min_pos.Width / 2 + rect6.Width / 2);
            Canvas.SetTop(text_min_pos, 5);
            text_min_pos.Text = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalStartPosition + "%";
            text_max_pos.Text = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalEndPosition + "%";
            Canvas.SetTop(rect7, 15);
            Canvas.SetLeft(rect7, rect7.Width / 2 + dx * dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalEndPosition);
            Canvas.SetLeft(text_max_pos, Canvas.GetLeft(rect7) - text_max_pos.Width / 2 + rect7.Width / 2);
            Canvas.SetTop(text_max_pos, 5);

            //set for RPM freq slider;
            dx = (canvas_horz_RPM_freq.Width - 10) / 50;
            Canvas.SetTop(rect_RPM_min, 15);

            Canvas.SetLeft(rect_RPM_min, rect_RPM_min.Width / 2 + dx * dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.RPM_min_freq);
            Canvas.SetLeft(text_RPM_freq_min, Canvas.GetLeft(rect_RPM_min) - text_RPM_freq_min.Width / 2 + rect_RPM_min.Width / 2);
            Canvas.SetTop(text_RPM_freq_min, 5);
            text_RPM_freq_min.Text = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.RPM_min_freq + "Hz";
            text_RPM_freq_max.Text = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.RPM_max_freq + "Hz";
            Canvas.SetTop(rect_RPM_max, 15);
            Canvas.SetLeft(rect_RPM_max, rect_RPM_max.Width / 2 + dx * dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.RPM_max_freq);
            Canvas.SetLeft(text_RPM_freq_max, Canvas.GetLeft(rect_RPM_max) - text_RPM_freq_max.Width / 2 + rect_RPM_max.Width / 2);
            Canvas.SetTop(text_RPM_freq_max, 5);
            //set for force vertical slider
            double dy = (canvas_vert_slider.Height / 250);
            Canvas.SetTop(rect8, canvas_vert_slider.Height - dy * dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.preloadForce);
            Canvas.SetLeft(rect8, canvas_vert_slider.Width / 2 - rect8.Width / 2 - Line_V_force.StrokeThickness / 2);
            Canvas.SetLeft(text_min_force, rect8.Width + 3);
            Canvas.SetTop(text_min_force, Canvas.GetTop(rect8));
            Canvas.SetTop(rect9, canvas_vert_slider.Height - dy * dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.maxForce);
            Canvas.SetLeft(rect9, canvas_vert_slider.Width / 2 - rect9.Width / 2 - Line_V_force.StrokeThickness / 2);
            Canvas.SetLeft(text_max_force, rect9.Width + 3);
            Canvas.SetTop(text_max_force, Canvas.GetTop(rect9) - 6 - text_max_force.Height / 2);

            text_min_force.Text = "Preload:\n" + dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.preloadForce + "kg";
            text_max_force.Text = "Max Force:\n" + dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.maxForce + "kg";
            //damping slider
            double damping_max = 255;
            dx = canvas_horz_damping.Width / damping_max;
            Canvas.SetLeft(rect_damping, dx * dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.dampingPress);
            text_damping.Text = "" + dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.dampingPress;
            Canvas.SetLeft(text_damping, Canvas.GetLeft(rect_damping) + rect_damping.Width / 2 - text_damping.Width / 2);
            Canvas.SetTop(text_damping, 5);
            //ABS amplitude slider
            double abs_max = 255;
            dx = canvas_horz_ABS.Width / abs_max;
            Canvas.SetLeft(rect_ABS, dx * dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.absAmplitude);
            switch (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.absForceOrTarvelBit)
            {
                case 0:
                    text_ABS.Text = (float)dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.absAmplitude / 20 + "kg";
                    break;
                case 1:
                    text_ABS.Text = (float)dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.absAmplitude / 20 + "%";
                    break;
                default:
                    break;
            }

            Canvas.SetLeft(text_ABS, Canvas.GetLeft(rect_ABS) - text_ABS.Width / 2 + rect_ABS.Width / 2);
            Canvas.SetTop(text_ABS, 5);
            //ABS freq slider
            double abs_freq_max = 30;
            dx = canvas_horz_ABS_freq.Width / abs_freq_max;
            Canvas.SetLeft(rect_ABS_freq, dx * dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.absFrequency);
            text_ABS_freq.Text = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.absFrequency + "Hz";
            Canvas.SetLeft(text_ABS_freq, Canvas.GetLeft(rect_ABS_freq) + rect_ABS_freq.Width / 2 - text_ABS_freq.Width / 2);
            Canvas.SetTop(text_ABS_freq, 5);

            // ABS pattern
            try
            {
                AbsPattern.SelectedIndex = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.absPattern;
            }
            catch (Exception caughtEx)
            {
            }

            // ABS force or travel dependent
            try
            {
                EffectAppliedOnForceOrTravel_combobox.SelectedIndex = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.absForceOrTarvelBit;
            }
            catch (Exception caughtEx)
            {
            }

            // spindle pitch
            try
            {
                SpindlePitch.SelectedIndex = (byte)dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.spindlePitch_mmPerRev_u8;
            }
            catch (Exception caughtEx)
            {
            }


            //max game output slider
            double max_game_max = 100;
            dx = canvas_horz_max_game.Width / max_game_max;
            Canvas.SetLeft(rect_max_game, dx * dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.maxGameOutput);
            text_max_game.Text = "" + dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.maxGameOutput + "%";
            Canvas.SetLeft(text_max_game, Canvas.GetLeft(rect_max_game) - text_max_game.Width / 2 + rect_max_game.Width / 2);
            Canvas.SetTop(text_max_game, 5);
            //KF SLider
            double KF_max = 255;
            dx = canvas_horz_KF.Width / KF_max;
            Canvas.SetLeft(rect_KF, dx * dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.kf_modelNoise);
            text_KF.Text = "" + dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.kf_modelNoise;
            Canvas.SetLeft(text_KF, Canvas.GetLeft(rect_KF) + rect_KF.Width / 2 - text_KF.Width / 2);
            Canvas.SetTop(text_KF, 5);
            //LC rating slider

            double LC_max = 510;
            dx = canvas_horz_LC_rating.Width / LC_max;
            Canvas.SetLeft(rect_LC_rating, dx * dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.loadcell_rating * 2);
            text_LC_rating.Text = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.loadcell_rating * 2 + "kg";
            Canvas.SetLeft(text_LC_rating, Canvas.GetLeft(rect_LC_rating) + rect_LC_rating.Width / 2 - text_LC_rating.Width / 2);
            Canvas.SetTop(text_LC_rating, 5);
            //RPM AMP slider

            double RPM_AMP_max = 200;
            dx = canvas_horz_RPM_AMP.Width / RPM_AMP_max;
            Canvas.SetLeft(rect_RPM_AMP, dx * dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.RPM_AMP);
            text_RPM_AMP.Text = ((float)dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.RPM_AMP) / 100 + "kg";
            Canvas.SetLeft(text_RPM_AMP, Canvas.GetLeft(rect_RPM_AMP) - text_RPM_AMP.Width / 2 + rect_RPM_AMP.Width / 2);
            Canvas.SetTop(text_RPM_AMP, 5);

            //Bite point control
            double BP_max = 100;
            dx = (double)canvas.Width / BP_max;
            text_BP.Text = "Bite Point:\n" + ((float)dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.BP_trigger_value) + "%";
            Canvas.SetLeft(rect_BP_Control, dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.BP_trigger_value * dx - rect_BP_Control.Width / 2);
            Canvas.SetLeft(text_BP, Canvas.GetLeft(rect_BP_Control) + rect_BP_Control.Width + 3);
            Canvas.SetTop(text_BP, canvas.Height - text_BP.Height);
            //Bite point freq slider
            double BP_freq_max = 30;
            dx = canvas_horz_bite_freq.Width / BP_freq_max;
            Canvas.SetLeft(rect_bite_freq, dx * dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.BP_freq);
            text_bite_freq.Text = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.BP_freq + "Hz";
            Canvas.SetLeft(text_bite_freq, Canvas.GetLeft(rect_bite_freq) + rect_bite_freq.Width / 2 - text_bite_freq.Width / 2);
            Canvas.SetTop(text_bite_freq, 5);

            //Bite point AMP slider
            double BP_amp_max = 200;
            dx = canvas_horz_bite_amp.Width / BP_amp_max;
            Canvas.SetLeft(rect_bite_amp, dx * dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.BP_amp);
            text_bite_amp.Text = ((float)dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.BP_amp) / 100.0f + "kg";
            Canvas.SetLeft(text_bite_amp, Canvas.GetLeft(rect_bite_amp) + rect_bite_amp.Width / 2 - text_bite_amp.Width / 2);
            Canvas.SetTop(text_bite_amp, 5);

            //Pgain slider
            double value_max = 2;
            dx = canvas_horz_Pgain.Width / value_max;
            Canvas.SetLeft(rect_Pgain, dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.PID_p_gain * dx);
            text_Pgain.Text = "" + Math.Round(dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.PID_p_gain, 2);
            Canvas.SetLeft(text_Pgain, Canvas.GetLeft(rect_Pgain) + rect_Pgain.Width / 2 - text_Pgain.Width / 2);
            Canvas.SetTop(text_Pgain, 5);

            //Igain slider

            value_max = 500;
            dx = canvas_horz_Igain.Width / value_max;
            Canvas.SetLeft(rect_Igain, dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.PID_i_gain * dx);
            text_Igain.Text = "" + Math.Round(dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.PID_i_gain, 2);
            Canvas.SetLeft(text_Igain, Canvas.GetLeft(rect_Igain) + rect_Igain.Width / 2 - text_Igain.Width / 2);
            Canvas.SetTop(text_Igain, 5);

            //Dgain slider
            value_max = 0.01;
            dx = canvas_horz_Dgain.Width / value_max;
            Canvas.SetLeft(rect_Dgain, dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.PID_d_gain * dx);
            text_Dgain.Text = "" + Math.Round(dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.PID_d_gain, 4);
            Canvas.SetLeft(text_Dgain, Canvas.GetLeft(rect_Dgain) + rect_Dgain.Width / 2 - text_Dgain.Width / 2);
            Canvas.SetTop(text_Dgain, 5);

            //VF gain slider
            value_max = 20;
            dx = canvas_horz_VFgain.Width / value_max;
            Canvas.SetLeft(rect_VFgain, dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.PID_velocity_feedforward_gain * dx);
            text_VFgain.Text = "" + Math.Round(dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.PID_velocity_feedforward_gain, 4);
            Canvas.SetLeft(text_VFgain, Canvas.GetLeft(rect_VFgain) + rect_VFgain.Width / 2 - text_VFgain.Width / 2);
            Canvas.SetTop(text_VFgain, 5);

            //MPC 0th order gain slider
            value_max = 4;
            dx = canvas_horz_MPC_0th_order_gain.Width / value_max;
            Canvas.SetLeft(rect_MPC_0th_order_gain, dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.MPC_0th_order_gain * dx);
            text_MPC_0th_order_gain.Text = "" + Math.Round(dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.MPC_0th_order_gain, 2) + "mm/kg";
            Canvas.SetLeft(text_MPC_0th_order_gain, Canvas.GetLeft(rect_MPC_0th_order_gain) + rect_MPC_0th_order_gain.Width / 2 - text_MPC_0th_order_gain.Width / 2);
            Canvas.SetTop(text_MPC_0th_order_gain, 5);


            ////MPC 1st order gain slider
            //value_max = 0.01;
            //dx = canvas_horz_MPC_1st_order_gain.Width / (2 * value_max);
            //Canvas.SetLeft(rect_MPC_1st_order_gain, dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.MPC_1st_order_gain * dx - value_max);
            //text_MPC_1st_order_gain.Text = "" + Math.Round(dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.MPC_1st_order_gain, 2);
            //Canvas.SetLeft(text_MPC_1st_order_gain, Canvas.GetLeft(rect_MPC_1st_order_gain) + rect_MPC_1st_order_gain.Width / 2 - rect_MPC_1st_order_gain.Width / 2);
            //Canvas.SetTop(text_MPC_1st_order_gain, 5);


            //G force multiplier slider
            double G_force_multi_max = 100;
            dx = canvas_horz_G_force_multi.Width / G_force_multi_max;
            Canvas.SetLeft(rect_G_force_multi, dx * dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.G_multi);
            text_G_multi.Text = (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.G_multi) + "%";
            Canvas.SetLeft(text_G_multi, Canvas.GetLeft(rect_G_force_multi) + rect_G_force_multi.Width / 2 - text_G_multi.Width / 2);
            Canvas.SetTop(text_G_multi, 5);

            //G force window slider
            value_max = 100;
            dx = canvas_horz_G_force_window.Width / value_max;
            Canvas.SetLeft(rect_G_force_window, dx * dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.G_window);
            text_G_window.Text = (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.G_window) + "";
            Canvas.SetLeft(text_G_window, Canvas.GetLeft(rect_G_force_window) + rect_G_force_window.Width / 2 - text_G_window.Width / 2);
            Canvas.SetTop(text_G_window, 5);

            //// Select serial port accordingly
            string tmp = (string)Plugin._serialPort[indexOfSelectedPedal_u].PortName;
            try
            {
                SerialPortSelection.SelectedValue = tmp;
                TextBox_debugOutput.Text = "Serial port selected: " + SerialPortSelection.SelectedValue;

            }
            catch (Exception caughtEx)
            {
            }


            if (Plugin._serialPort[indexOfSelectedPedal_u].IsOpen == true)
            {
                ConnectToPedal.IsChecked = true;
                btn_pedal_connect.Content = "Disconnect From Pedal";
            }
            else
            {
                ConnectToPedal.IsChecked = false;
                btn_pedal_connect.Content = "Connect To Pedal";
            }

            if (Plugin.Settings.RPM_enable_flag[indexOfSelectedPedal_u] == 1)
            {
                checkbox_enable_RPM.IsChecked = true;
                checkbox_enable_RPM.Content = "Effect Enabled";
            }
            else
            {
                checkbox_enable_RPM.IsChecked = false;
                checkbox_enable_RPM.Content = "Effect Disabled";
            }

            if (Plugin.Settings.ABS_enable_flag[indexOfSelectedPedal_u] == 1)
            {
                checkbox_enable_ABS.IsChecked = true;
                checkbox_enable_ABS.Content = "ABS/TC Effect Enabled";
            }
            else
            {
                checkbox_enable_ABS.IsChecked = false;
                checkbox_enable_ABS.Content = "ABS/TC Effect Disabled";
            }
            if (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.BP_trigger == 1)
            {
                checkbox_enable_bite_point.IsChecked = true;
                text_BP.Visibility = Visibility.Visible;
                rect_BP_Control.Visibility = Visibility.Visible;
                checkbox_enable_bite_point.Content = "Bite Point Vibration Enabled";

            }
            else
            {
                checkbox_enable_bite_point.IsChecked = false;
                text_BP.Visibility = Visibility.Hidden;
                rect_BP_Control.Visibility = Visibility.Hidden;
                checkbox_enable_bite_point.Content = "Bite Point Vibration Disabled";
            }

            if (indexOfSelectedPedal_u == 1)
            {
                checkbox_enable_G_force.IsEnabled = true;
                if (Plugin.Settings.G_force_enable_flag[indexOfSelectedPedal_u] == 1)
                {
                    checkbox_enable_G_force.IsChecked = true;
                    checkbox_enable_G_force.Content = "G Force Effect Enabled";
                }
                else
                {
                    checkbox_enable_G_force.IsChecked = false;
                    checkbox_enable_G_force.Content = "G Force Effect Disabled";
                }
            }
            else
            {
                checkbox_enable_G_force.IsEnabled = false;
                checkbox_enable_G_force.IsChecked = false;
                checkbox_enable_G_force.Content = "G Force Effect Disabled";
            }

            if (Plugin.Settings.connect_status[indexOfSelectedPedal_u] == 1)
            {
                Serial_port_text.Text = "Serial Port connected";
                Serial_port_text.Visibility = Visibility.Visible;

            }
            else
            {
                Serial_port_text.Visibility = Visibility.Hidden;
            }

            if (Plugin.Settings.RPM_effect_type == 0)
            {
                RPMeffecttype_Sel_1.IsChecked = true;
            }
            else
            {
                RPMeffecttype_Sel_2.IsChecked = true;
            }

            if (Plugin.Settings.file_enable[profile_select, 0] == 1)
            {
                Label_clutch_file.Content = Plugin.Settings.pedal_file_string[profile_select,0];
                Clutch_file_check.IsChecked = true;
            }
            else 
            {
                Label_clutch_file.Content = "";
                Clutch_file_check.IsChecked = false;
            }
            if (Plugin.Settings.file_enable[profile_select, 1] == 1)
            {
                Label_brake_file.Content = Plugin.Settings.pedal_file_string[profile_select, 1];
                Brake_file_check.IsChecked = true;
            }
            else
            {
                Label_brake_file.Content = "";
                Brake_file_check.IsChecked = false;
            }

            if (Plugin.Settings.file_enable[profile_select,2] == 1)
            {
                Label_gas_file.Content = Plugin.Settings.pedal_file_string[profile_select, 2];
                Gas_file_check.IsChecked = true;
            }
            else
            {
                Label_gas_file.Content = "";
                Gas_file_check.IsChecked = false;
            }


            //TextBox2.Text = "" + Plugin.Settings.selectedComPortNames[0] + Plugin.Settings.selectedComPortNames[1] + Plugin.Settings.selectedComPortNames[2];
            JoystickOutput_check.IsChecked = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.travelAsJoystickOutput_u8 == 1;
            InvertLoadcellReading_check.IsChecked = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.invertLoadcellReading_u8 == 1;
            InvertMotorDir_check.IsChecked = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.invertMotorDirection_u8 == 1;

            
            Label_vjoy_order.Content = Plugin.Settings.vjoy_order;

            //try
            //{
            //    //ComboBox_JsonFileSelected.SelectedItem = Plugin.Settings.selectedJsonFileNames[indexOfSelectedPedal_u];
            //    //ComboBox_JsonFileSelected.SelectedValue = (string)Plugin.Settings.selectedJsonFileNames[indexOfSelectedPedal_u];

            //    ComboBox_JsonFileSelected.SelectedIndex = Plugin.Settings.selectedJsonIndexLast[indexOfSelectedPedal_u];

            //    //ReadStructFromJson();


            //    //SerialPortSelection.SelectedValue
            //    //TextBox_debugOutput.Text = "Error 2: ";
            //    //TextBox_debugOutput.Text += Plugin.Settings.selectedJsonFileNames[indexOfSelectedPedal_u];
            //    //TextBox_debugOutput.Text += "     ";
            //    //TextBox_debugOutput.Text += ComboBox_JsonFileSelected.SelectedValue;
            //}
            //catch (Exception caughtEx)
            //{
            //    string errorMessage = caughtEx.Message;
            //    TextBox_debugOutput.Text = "Error 1: ";
            //    TextBox_debugOutput.Text += errorMessage;
            //}

            //= ComboBox_JsonFileSelected.SelectedItem.ToString();

            //ConnectToPedal.IsChecked = true;

            //TextBox_debugOutput.Text = "Pedal selected: " + indexOfSelectedPedal_u;
            //TextBox_debugOutput.Text += ",    connected: " + ConnectToPedal.IsChecked;
            //TextBox_debugOutput.Text += ",    serial port name: " + tmp;

        }




        private void Update_BrakeForceCurve()
        {

            double[] x = new double[6];
            double[] y = new double[6];
            double x_quantity = 100;
            double y_max = 100;
            double dx = canvas.Width / x_quantity;
            double dy = canvas.Height / y_max;

            x[0] = 0;
            x[1] = 20;
            x[2] = 40;
            x[3] = 60;
            x[4] = 80;
            x[5] = 100;

            y[0] = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.relativeForce_p000;
            y[1] = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.relativeForce_p020;
            y[2] = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.relativeForce_p040;
            y[3] = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.relativeForce_p060;
            y[4] = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.relativeForce_p080;
            y[5] = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.relativeForce_p100;

            // Use cubic interpolation to smooth the original data
            (double[] xs2, double[] ys2, double[] a, double[] b) = Cubic.Interpolate1D(x, y, 100);


            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.cubic_spline_param_a_0 = (float)a[0];
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.cubic_spline_param_a_1 = (float)a[1];
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.cubic_spline_param_a_2 = (float)a[2];
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.cubic_spline_param_a_3 = (float)a[3];
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.cubic_spline_param_a_4 = (float)a[4];

            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.cubic_spline_param_b_0 = (float)b[0];
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.cubic_spline_param_b_1 = (float)b[1];
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.cubic_spline_param_b_2 = (float)b[2];
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.cubic_spline_param_b_3 = (float)b[3];
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.cubic_spline_param_b_4 = (float)b[4];


            //TextBox_debugOutput.Text = "";
            //for (uint i = 0; i < a.Length; i++)
            //{
            //    TextBox_debugOutput.Text += "\na[" + i + "]: " + a[i] + "      b[" + i + "]: " + b[i];
            //}


            System.Windows.Media.PointCollection myPointCollection2 = new System.Windows.Media.PointCollection();


            for (int pointIdx = 0; pointIdx < 100; pointIdx++)
            {
                System.Windows.Point Pointlcl = new System.Windows.Point(dx * xs2[pointIdx], dy * ys2[pointIdx]);
                myPointCollection2.Add(Pointlcl);
                Force_curve_Y[pointIdx] = dy * ys2[pointIdx];
            }

            this.Polyline_BrakeForceCurve.Points = myPointCollection2;


        }



        public class SerialPortChoice
        {
            public SerialPortChoice(string display, string value)
            {
                Display = display;
                Value = value;
            }

            public string Value { get; set; }
            public string Display { get; set; }
        }



        // Select which pedal to config
        // see https://stackoverflow.com/questions/772841/is-there-selected-tab-changed-event-in-the-standard-wpf-tab-control
        private void TabControl_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            indexOfSelectedPedal_u = (uint)MyTab.SelectedIndex;
            Plugin.Settings.table_selected = (uint)MyTab.SelectedIndex;
            // update the sliders & serial port selection accordingly
            updateTheGuiFromConfig();
        }





        /********************************************************************************************************************/
        /*							Slider callbacks																		*/
        /********************************************************************************************************************/

        public void TestAbs_click(object sender, RoutedEventArgs e)
        {
            //if (indexOfSelectedPedal_u == 1)
            if (TestAbs_check.IsChecked == false)
            {
                TestAbs_check.IsChecked = true;
                Plugin.sendAbsSignal = (bool)TestAbs_check.IsChecked;
                TextBox_debugOutput.Text = "ABS-Test begin";
                updateTheGuiFromConfig();
            }
            else
            {
                TestAbs_check.IsChecked = false;
                //Plugin.sendAbsSignal = !Plugin.sendAbsSignal;
                Plugin.sendAbsSignal = (bool)TestAbs_check.IsChecked;
                TextBox_debugOutput.Text = "ABS-Test stopped";
                updateTheGuiFromConfig();
            }

        }









        /********************************************************************************************************************/
        /*							PID tuning                      														*/
        /********************************************************************************************************************/
        public void PID_tuning_P_gain_changed(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.PID_p_gain = (float)e.NewValue;
        }

        public void PID_tuning_I_gain_changed(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.PID_i_gain = (float)e.NewValue;
        }

        public void PID_tuning_D_gain_changed(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.PID_d_gain = (float)e.NewValue;
        }

        public void PID_tuning_Feedforward_gain_changed(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.PID_velocity_feedforward_gain = (float)e.NewValue;
        }







        private void NumericTextBox_TextChanged(object sender, TextChangedEventArgs e)
        {
            //labelEingabe.Content = "Sie haben '" + textBox_debug_Flag_0.Text + "' eingegeben!";
            //TextBox_debugOutput.Text = textBox_debug_Flag_0.Text;

            if (int.TryParse(textBox_debug_Flag_0.Text, out int result))
            {
                if ((result >= 0) && (result <= 255))
                {
                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.debug_flags_0 = (byte)result;
                }
            }
        }
        private void NumericTextBox_PreviewTextInput(object sender, TextCompositionEventArgs e)
        {

            //if ((e.NewValue >= 0) && (e.NewValue <= 255))
            //{
            //    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.debug_flags_0 = (byte)e.NewValue;
            //}

            // Use a regular expression to allow only numeric input
            Regex regex = new Regex("[^0-9]+");

            System.Windows.Controls.TextBox textBox = (System.Windows.Controls.TextBox)sender;

            e.Handled = regex.IsMatch(textBox.Text + e.Text);

            ////if (!e.Handled)
            ////{
            ////    if (int.TryParse(textBox.Text + e.Text, out int result))
            ////    {
            ////        if ((result >= 0) && (result <= 255))
            ////        {
            ////            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.debug_flags_0 = (byte)result;
            ////        }
            ////    }
            ////}
        }


        /********************************************************************************************************************/
        /*							Write/read config to/from Json file														*/
        /********************************************************************************************************************/

        //private void ComboBox_SelectionChanged(object sender, EventArgs e)
        //{
        //    try
        //    {
        //        // https://stackoverflow.com/questions/3721430/what-is-the-simplest-way-to-get-the-selected-text-of-a-combo-box-containing-only

        //        string stringValue = ((ComboBoxItem)ComboBox_JsonFileSelected.SelectedItem).Content.ToString();


        //        // string stringValue = ComboBox_JsonFileSelected.SelectedValue.ToString();

        //        //TextBox_debugOutput.Text = stringValue;
        //        Plugin.Settings.selectedJsonFileNames[indexOfSelectedPedal_u] = stringValue;

        //        Plugin.Settings.selectedJsonIndexLast[indexOfSelectedPedal_u] = ComboBox_JsonFileSelected.SelectedIndex;



        //        //ReadStructFromJson();
        //    }
        //    catch (Exception caughtEx)
        //    {

        //        string errorMessage = caughtEx.Message;
        //        TextBox_debugOutput.Text = errorMessage;
        //    }
        //}




        //public void SaveStructToJson_click(object sender, RoutedEventArgs e)
        //{
        //    // https://learn.microsoft.com/en-us/dotnet/standard/serialization/system-text-json/how-to?pivots=dotnet-8-0

        //    try
        //    {
        //        // which config file is seleced
        //        string currentDirectory = Directory.GetCurrentDirectory();
        //        string dirName = currentDirectory + "\\PluginsData\\Common";
        //        string jsonFileName = ComboBox_JsonFileSelected.Text;
        //        string fileName = dirName + "\\" + jsonFileName + ".json";

        //        this.dap_config_st[indexOfSelectedPedal_u].payloadHeader_.version = (byte)pedalConfigPayload_version;

        //        // https://stackoverflow.com/questions/3275863/does-net-4-have-a-built-in-json-serializer-deserializer
        //        // https://learn.microsoft.com/en-us/dotnet/framework/wcf/feature-details/how-to-serialize-and-deserialize-json-data?redirectedfrom=MSDN
        //        var stream1 = new MemoryStream();
        //        var ser = new DataContractJsonSerializer(typeof(DAP_config_st));
        //        ser.WriteObject(stream1, dap_config_st[indexOfSelectedPedal_u]);

        //        stream1.Position = 0;
        //        StreamReader sr = new StreamReader(stream1);
        //        string jsonString = sr.ReadToEnd();

        //        // Check if file already exists. If yes, delete it.     
        //        if (File.Exists(fileName))
        //        {
        //            File.Delete(fileName);
        //        }


        //        System.IO.File.WriteAllText(fileName, jsonString);
        //        TextBox_debugOutput.Text = "Config exported!";

        //    }
        //    catch (Exception caughtEx)
        //    {

        //        string errorMessage = caughtEx.Message;
        //        TextBox_debugOutput.Text = errorMessage;
        //    }

        //}



        //public void ReadStructFromJson_click(object sender, RoutedEventArgs e)
        //{
        //    ReadStructFromJson();
        //}


        /********************************************************************************************************************/
        /*							Refind min endstop																		*/
        /********************************************************************************************************************/
        unsafe public void ResetPedalPosition_click(object sender, RoutedEventArgs e)
        {

            if (Plugin._serialPort[indexOfSelectedPedal_u].IsOpen)
            {

                try
                {
                    // compute checksum
                    DAP_action_st tmp;
                    tmp.payloadHeader_.version = (byte)Constants.pedalConfigPayload_version;
                    tmp.payloadHeader_.payloadType = (byte)Constants.pedalActionPayload_type;
                    tmp.payloadPedalAction_.resetPedalPos_u8 = 1;


                    DAP_action_st* v = &tmp;
                    byte* p = (byte*)v;
                    tmp.payloadFooter_.checkSum = Plugin.checksumCalc(p, sizeof(payloadHeader) + sizeof(payloadPedalAction));


                    int length = sizeof(DAP_action_st);
                    byte[] newBuffer = new byte[length];
                    newBuffer = Plugin.getBytes_Action(tmp);


                    // clear inbuffer 
                    Plugin._serialPort[indexOfSelectedPedal_u].DiscardInBuffer();

                    // send query command
                    Plugin._serialPort[indexOfSelectedPedal_u].Write(newBuffer, 0, newBuffer.Length);
                }
                catch (Exception caughtEx)
                {
                    string errorMessage = caughtEx.Message;
                    TextBox_debugOutput.Text = errorMessage;
                }
            }
        }



        /********************************************************************************************************************/
        /*							System identification																	*/
        /********************************************************************************************************************/
        public void StartSystemIdentification_click(object sender, RoutedEventArgs e)
        {

            TextBox_debugOutput.Text = "Start system identification";


            try
            {

                string currentDirectory = Directory.GetCurrentDirectory();
                string dirName = currentDirectory + "\\PluginsData\\Common";
                string logFileName = "DiyActivePedalSystemIdentification";
                string fileName = dirName + "\\" + logFileName + ".txt";

                if (File.Exists(fileName))
                {
                    File.Delete(fileName);
                }


                // This text is added only once to the file.
                if (!File.Exists(fileName))
                {
                    using (StreamWriter sw = File.CreateText(fileName))
                    {

                        // trigger system identification
                        Plugin._serialPort[indexOfSelectedPedal_u].Write("3");

                        //System.Threading.Thread.Sleep(100);


                        // read system return log
                        //while (Plugin._serialPort[indexOfSelectedPedal_u].BytesToRead > 0)
                        //{
                        //    string message = Plugin._serialPort[indexOfSelectedPedal_u].ReadLine();
                        //    sw.Write(message);

                        //    System.Threading.Thread.Sleep(20);

                        //}
                    }

                }

                TextBox_debugOutput.Text = "Finished system identification";


                ////// trigger system identification
                ////Plugin._serialPort[indexOfSelectedPedal_u].Write("3");

                ////System.Threading.Thread.Sleep(100);


                ////// read system return log
                ////while (Plugin._serialPort[indexOfSelectedPedal_u].BytesToRead > 0)
                ////{
                ////    string message = Plugin._serialPort[indexOfSelectedPedal_u].ReadLine();
                ////    using (StreamWriter sw = File.AppendText(fileName))
                ////    {
                ////        sw.WriteLine(message);
                ////    }
                ////    System.Threading.Thread.Sleep(100);
                ////}


            }
            catch (Exception caughtEx)
            {
                string errorMessage = caughtEx.Message;
                TextBox_debugOutput.Text = errorMessage;
            }

        }






        /********************************************************************************************************************/
        /*							Send config to pedal																	*/
        /********************************************************************************************************************/
        unsafe public void Sendconfig(uint pedalIdx)
        {


            if (Plugin._serialPort[pedalIdx].IsOpen)
            {

                // compute checksum
                //getBytes(this.dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_)
                this.dap_config_st[pedalIdx].payloadHeader_.version = (byte)Constants.pedalConfigPayload_version;
                this.dap_config_st[pedalIdx].payloadHeader_.payloadType = (byte)Constants.pedalConfigPayload_type;
                this.dap_config_st[pedalIdx].payloadHeader_.storeToEeprom = 1;
                DAP_config_st tmp = this.dap_config_st[pedalIdx];

                //payloadPedalConfig tmp = this.dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_;
                DAP_config_st* v = &tmp;
                byte* p = (byte*)v;
                this.dap_config_st[pedalIdx].payloadFooter_.checkSum = Plugin.checksumCalc(p, sizeof(payloadHeader) + sizeof(payloadPedalConfig));


                //TextBox_debugOutput.Text = "CRC simhub calc: " + this.dap_config_st[indexOfSelectedPedal_u].payloadFooter_.checkSum + "    ";

                TextBox_debugOutput.Text = String.Empty;

                try
                {
                    int length = sizeof(DAP_config_st);
                    //int val = this.dap_config_st[indexOfSelectedPedal_u].payloadHeader_.checkSum;
                    //string msg = "CRC value: " + val.ToString();
                    byte[] newBuffer = new byte[length];
                    newBuffer = getBytes(this.dap_config_st[pedalIdx]);

                    //TextBox_debugOutput.Text = "ConfigLength" + length;

                    // clear inbuffer 
                    Plugin._serialPort[pedalIdx].DiscardInBuffer();
                    Plugin._serialPort[pedalIdx].DiscardOutBuffer();


                    // send data
                    Plugin._serialPort[pedalIdx].Write(newBuffer, 0, newBuffer.Length);
                    //Plugin._serialPort[indexOfSelectedPedal_u].Write("\n");
                }
                catch (Exception caughtEx)
                {
                    string errorMessage = caughtEx.Message;
                    TextBox_debugOutput.Text = errorMessage;
                }

            }
        }
        unsafe public void Sendconfigtopedal_shortcut()
        {

            for (uint pedalIdx = 0; pedalIdx < 3; pedalIdx++)
            {
                if (Plugin.Settings.file_enable[profile_select, pedalIdx] == 1)
                {
                    Sendconfig(pedalIdx);
                    TextBox_debugOutput.Text = "config was sent to pedal";
                }
            }

        }
        unsafe public void SendConfigToPedal_click(object sender, RoutedEventArgs e)
        {

            Sendconfig(indexOfSelectedPedal_u);
            /*
            if (Plugin._serialPort[indexOfSelectedPedal_u].IsOpen)
            {

                // compute checksum
                //getBytes(this.dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_)
                this.dap_config_st[indexOfSelectedPedal_u].payloadHeader_.version = (byte)Constants.pedalConfigPayload_version;
                this.dap_config_st[indexOfSelectedPedal_u].payloadHeader_.payloadType = (byte)Constants.pedalConfigPayload_type;
                this.dap_config_st[indexOfSelectedPedal_u].payloadHeader_.storeToEeprom = 1;
                DAP_config_st tmp = this.dap_config_st[indexOfSelectedPedal_u];

                //payloadPedalConfig tmp = this.dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_;
                DAP_config_st* v = &tmp;
                byte* p = (byte*)v;
                this.dap_config_st[indexOfSelectedPedal_u].payloadFooter_.checkSum = Plugin.checksumCalc(p, sizeof(payloadHeader) + sizeof(payloadPedalConfig));


                //TextBox_debugOutput.Text = "CRC simhub calc: " + this.dap_config_st[indexOfSelectedPedal_u].payloadFooter_.checkSum + "    ";

                TextBox_debugOutput.Text = String.Empty;

                try
                {
                    int length = sizeof(DAP_config_st);
                    //int val = this.dap_config_st[indexOfSelectedPedal_u].payloadHeader_.checkSum;
                    //string msg = "CRC value: " + val.ToString();
                    byte[] newBuffer = new byte[length];
                    newBuffer = getBytes(this.dap_config_st[indexOfSelectedPedal_u]);

                    //TextBox_debugOutput.Text = "ConfigLength" + length;

                    // clear inbuffer 
                    Plugin._serialPort[indexOfSelectedPedal_u].DiscardInBuffer();
                    Plugin._serialPort[indexOfSelectedPedal_u].DiscardOutBuffer();


                    // send data
                    Plugin._serialPort[indexOfSelectedPedal_u].Write(newBuffer, 0, newBuffer.Length);
                    //Plugin._serialPort[indexOfSelectedPedal_u].Write("\n");
                }
                catch (Exception caughtEx)
                {
                    string errorMessage = caughtEx.Message;
                    TextBox_debugOutput.Text = errorMessage;
                }

            }
            */
        }


        unsafe public void Reading_config_auto(uint i)
        {
            if (Plugin._serialPort[i].IsOpen)
            {
                // compute checksum
                DAP_action_st tmp;
                tmp.payloadPedalAction_.returnPedalConfig_u8 = 1;
                tmp.payloadHeader_.version = (byte)Constants.pedalConfigPayload_version;
                tmp.payloadHeader_.payloadType = (byte)Constants.pedalActionPayload_type;

                DAP_action_st* v = &tmp;
                byte* p = (byte*)v;
                tmp.payloadFooter_.checkSum = Plugin.checksumCalc(p, sizeof(payloadHeader) + sizeof(payloadPedalAction));


                int length = sizeof(DAP_action_st);
                byte[] newBuffer = new byte[length];
                newBuffer = Plugin.getBytes_Action(tmp);


                // tell the plugin that we expect config data
                waiting_for_pedal_config[i] = true;


                // try N times and check whether config has been received
                for (int rep = 0; rep < 1; rep++)
                {
                    // send query command
                    Plugin._serialPort[i].Write(newBuffer, 0, newBuffer.Length);

                    // wait some time and check whether data has been received
                    System.Threading.Thread.Sleep(50);

                    if (waiting_for_pedal_config[i] == false)
                    {
                        break;
                    }
                }
            }
        }

        public string[] STOPCHAR = { "\r\n" };
        public bool EndsWithStop(string incomingData)
        {
            for (int i = 0; i < STOPCHAR.Length; i++)
            {
                if (incomingData.EndsWith(STOPCHAR[i]))
                {
                    return true;
                }
            }
            return false;
        }

        /********************************************************************************************************************/
        /*							Read config from pedal																	*/
        /********************************************************************************************************************/
        unsafe public void ReadConfigFromPedal_click(object sender, RoutedEventArgs e)
        {
            Reading_config_auto(indexOfSelectedPedal_u);
        }


        public string[] _data = { "", "", "" };// = "";

        //unsafe private void sp_DataReceived(object sender, object e)
        unsafe private void sp_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {

            SerialPort sp = (SerialPort)sender;
            //string _type = (string)e;

            //if (Plugin._serialPort[indexOfSelectedPedal_u].PortName = sp.PortName)

            // identify which pedal has send the data
            int pedalSelected = 255;
            for (int pedalIdx_i = 0; pedalIdx_i < 3; pedalIdx_i++)
            {
                if ((Plugin._serialPort[pedalIdx_i].PortName == sp.PortName) && (Plugin._serialPort[pedalIdx_i].IsOpen))
                {
                    pedalSelected = pedalIdx_i;
                }
            }

            // once the pedal has identified, go ahead
            if (pedalSelected < 3)
            //if (Plugin._serialPort[indexOfSelectedPedal_u].IsOpen)
            {
                // https://stackoverflow.com/questions/9732709/the-calling-thread-cannot-access-this-object-because-a-different-thread-owns-it


                int length = sizeof(DAP_config_st);
                byte[] newBuffer_config = new byte[length];

                int receivedLength = sp.BytesToRead;


                string incomingData = sp.ReadExisting();
                //if the data doesn't end with a stop char this will signal to keep it in _data 
                //for appending to the following read of data
                bool endsWithStop = EndsWithStop(incomingData);

                //each array object will be sent separately to the callback
                string[] dataArray = incomingData.Split(STOPCHAR, StringSplitOptions.None);

                for (int i = 0; i < dataArray.Length; i++)
                {
                    string newData = dataArray[i];

                    //if you are at the last object in the array and this hasn't got a stopchar after
                    //it will be saved in _data
                    if (!endsWithStop && i == dataArray.Length - 1)
                    {
                        _data[pedalSelected] += newData;
                    }
                    else
                    {
                        string dataToSend = _data[pedalSelected] + newData;
                        _data[pedalSelected] = "";


                        // decode into config struct
                        if (dataToSend.Length == length)
                        {
                            DAP_config_st tmp;

                            // transform string into byte
                            fixed (byte* p = Encoding.ASCII.GetBytes(dataToSend))
                            {
                                // create a fixed size buffer
                                length = sizeof(DAP_config_st);
                                byte[] newBuffer_config_2 = new byte[length];

                                // copy the received bytes into byte array
                                for (int j = 0; j < length; j++)
                                {
                                    newBuffer_config_2[j] = p[j];
                                }

                                // parse byte array as config struct
                                DAP_config_st pedalConfig_read_st = getConfigFromBytes(newBuffer_config_2);

                                // check whether receive struct is plausible
                                DAP_config_st* v_config = &pedalConfig_read_st;
                                byte* p_config = (byte*)v_config;

                                // payload type check
                                bool check_payload_config_b = false;
                                if (pedalConfig_read_st.payloadHeader_.payloadType == Constants.pedalConfigPayload_type)
                                {
                                    check_payload_config_b = true;
                                }

                                // CRC check
                                bool check_crc_config_b = false;
                                if (Plugin.checksumCalc(p_config, sizeof(payloadHeader) + sizeof(payloadPedalConfig)) == pedalConfig_read_st.payloadFooter_.checkSum)
                                {
                                    check_crc_config_b = true;
                                }


                                // when all checks are passed, accept the config. Otherwise discard and trow error
                                Dispatcher.Invoke(
                                new Action<DAP_config_st>((t) => this.dap_config_st[pedalSelected] = t),
                                pedalConfig_read_st);


                                this.Dispatcher.Invoke(() =>
                                {
                                    // update pedal config
                                    if (check_payload_config_b)
                                    {
                                        //this.dap_config_st[indexOfSelectedPedal_u] = pedalConfig_read_st;
                                        updateTheGuiFromConfig();
                                    }

                                    TextBox_debugOutput.Text = "Payload config test 1: " + check_payload_config_b;
                                    TextBox_debugOutput.Text += "Payload config test 2: " + check_crc_config_b;
                                });

                            }

                        }
                        else
                        {
                            this.Dispatcher.Invoke(() =>
                            {
                                //TextBox_serialMonitor.Text += "DataArrayLength: " + dataArray.Length + "\n";
                                //TextBox_serialMonitor.Text += "DataLength: " + dataToSend.Length + "\n";
                                TextBox_serialMonitor.Text += dataToSend + "\n";

                                TextBox_serialMonitor.ScrollToEnd();
                                //TextBox_serialMonitor.Text += receivedLength + "\n";
                            });
                        }


                    }











                    //limits the data stored to 1000 to avoid using up all the memory in case of 
                    //failure to register callback or include stopchar

                    if (_data[pedalSelected].Length > 1000)
                    {
                        _data[pedalSelected] = "";
                    }


                    //////this.Dispatcher.Invoke(() =>
                    //////    {
                    //////        TextBox_serialMonitor.Text += incomingData;

                    //////        TextBox_serialMonitor.ScrollToEnd();
                    //////        //TextBox_serialMonitor.Text += receivedLength + "\n";
                    //////    });
                }

                // obtain data and check whether it is from known payload type or just debug info

            }
        }



        /********************************************************************************************************************/
        /*							read serial stream																		*/
        /********************************************************************************************************************/
        public void openSerialAndAddReadCallback(uint pedalIdx)
        {
            try
            {
                // serial port settings
                Plugin._serialPort[pedalIdx].Handshake = Handshake.None;
                Plugin._serialPort[pedalIdx].Parity = Parity.None;
                //_serialPort[pedalIdx].StopBits = StopBits.None;


                Plugin._serialPort[pedalIdx].ReadTimeout = 2000;
                Plugin._serialPort[pedalIdx].WriteTimeout = 500;

                // https://stackoverflow.com/questions/7178655/serialport-encoding-how-do-i-get-8-bit-ascii
                Plugin._serialPort[pedalIdx].Encoding = System.Text.Encoding.GetEncoding(28591);

                Plugin._serialPort[pedalIdx].DtrEnable = false;

                Plugin._serialPort[pedalIdx].NewLine = "\r\n";
                Plugin._serialPort[pedalIdx].ReadBufferSize = 10000;
                if (Plugin.Settings.auto_connect_flag == 1 & Plugin.Settings.connect_flag[pedalIdx] == 1 )
                {
                    if (Plugin.Settings.autoconnectComPortNames[pedalIdx] == "NA")
                    {
                        Plugin._serialPort[pedalIdx].PortName = Plugin.Settings.autoconnectComPortNames[pedalIdx];
                    }
                    else
                    {
                        Plugin._serialPort[pedalIdx].PortName = Plugin.Settings.selectedComPortNames[pedalIdx];
                        Plugin.Settings.autoconnectComPortNames[pedalIdx] = Plugin.Settings.selectedComPortNames[pedalIdx];
                    }
                    
                }
                else
                {
                    Plugin._serialPort[pedalIdx].PortName = Plugin.Settings.selectedComPortNames[pedalIdx];
                    Plugin.Settings.autoconnectComPortNames[pedalIdx]= Plugin.Settings.selectedComPortNames[pedalIdx];
                }
                
                if (Plugin.PortExists(Plugin._serialPort[pedalIdx].PortName))
                {
                    Plugin._serialPort[pedalIdx].Open();
                    Plugin.Settings.connect_status[pedalIdx] = 1;
                    // read callback
                    pedal_serial_read_timer[pedalIdx] = new System.Windows.Forms.Timer();
                    pedal_serial_read_timer[pedalIdx].Tick += new EventHandler(timerCallback_serial);
                    pedal_serial_read_timer[pedalIdx].Tag = pedalIdx;
                    pedal_serial_read_timer[pedalIdx].Interval = 16; // in miliseconds
                    pedal_serial_read_timer[pedalIdx].Start();
                    System.Threading.Thread.Sleep(100);
                }
                else
                {
                    Plugin.Settings.connect_status[pedalIdx] = 0;
                    Plugin.connectSerialPort[pedalIdx] = false;

                }
            }
            catch (Exception ex)
            { }




        }
        private uint count_timmer_count = 0;
        public void connection_timmer_tick(object sender, EventArgs e)
        {
            if (Plugin.profile_update_flag == 1)
            {
                Profile_change(Plugin.profile_index);
                Plugin.profile_update_flag = 0;
            }

            count_timmer_count++;
            if (count_timmer_count > 1)
            {
                if (Plugin.Settings.auto_connect_flag == 1)
                {
                    for (uint pedalIdx = 0; pedalIdx < 3; pedalIdx++)
                    {


                        if (Plugin.Settings.connect_flag[pedalIdx] == 1)
                        {
                            if (Plugin.PortExists(Plugin._serialPort[pedalIdx].PortName))
                            {
                                if (Plugin._serialPort[pedalIdx].IsOpen == false)
                                {
                                    //UpdateSerialPortList_click();
                                    openSerialAndAddReadCallback(pedalIdx);
                                    //Plugin.Settings.autoconnectComPortNames[pedalIdx] = Plugin._serialPort[pedalIdx].PortName;
                                    if (Plugin.Settings.reading_config == 1)
                                    {
                                        Reading_config_auto(pedalIdx);


                                    }
                                    updateTheGuiFromConfig();
                                }
                            }
                            else
                            {
                                Plugin.connectSerialPort[pedalIdx] = false;
                                Plugin.Settings.connect_status[pedalIdx] = 0;
                                updateTheGuiFromConfig();
                            }




                        }
                    }

                }
            }
            if (count_timmer_count > 200)
            {
                count_timmer_count = 2;
            }

        }

        public void closeSerialAndStopReadCallback(uint pedalIdx)
        {
            if (pedal_serial_read_timer[pedalIdx] != null)
            {
                pedal_serial_read_timer[pedalIdx].Stop();
                pedal_serial_read_timer[pedalIdx].Dispose();
            }
            connect_timer.Dispose();
            connect_timer.Stop();
            System.Threading.Thread.Sleep(300);
            if (Plugin._serialPort[pedalIdx].IsOpen)
            {
                Plugin._serialPort[pedalIdx].DiscardInBuffer();
                Plugin._serialPort[pedalIdx].DiscardOutBuffer();
                Plugin._serialPort[pedalIdx].Close();
                Plugin.Settings.connect_status[pedalIdx] = 0;
            }
        }

        Int64 writeCntr = 0;

        int[] timeCntr = { 0, 0, 0 };

        double[] timeCollector = { 0, 0, 0 };


        static List<int> FindAllOccurrences(byte[] source, byte[] sequence, int maxLength)
        {
            List<int> indices = new List<int>();

            int len = source.Length - sequence.Length;
            if (len > maxLength)
            {
                len = maxLength;
            }

            for (int i = 0; i <= len; i++)
            {
                bool found = true;
                for (int j = 0; j < sequence.Length; j++)
                {
                    if (source[i + j] != sequence[j])
                    {
                        found = false;
                        break;
                    }
                }
                if (found)
                {
                    indices.Add(i); // Sequence found, add index to the list
                }
            }

            return indices;
        }


        int[] appendedBufferOffset = { 0, 0, 0 };

        static int bufferSize = 100000;
        byte[][] buffer_appended = { new byte[bufferSize], new byte[bufferSize], new byte[bufferSize] };

        unsafe public void timerCallback_serial(object sender, EventArgs e)
        {

            //action here 

            if (Plugin.profile_update_flag == 1)
            {
                Profile_change(Plugin.profile_index);
                Plugin.profile_update_flag = 0;
            }

            if (Plugin.sendconfig_flag == 1)
            {
                Sendconfigtopedal_shortcut();
                Plugin.sendconfig_flag = 0;
            }


            int pedalSelected = Int32.Parse((sender as System.Windows.Forms.Timer).Tag.ToString());
            //int pedalSelected = (int)(sender as System.Windows.Forms.Timer).Tag;

            bool pedalStateHasAlreadyBeenUpdated_b = false;

            // once the pedal has identified, go ahead
            if (pedalSelected < 3)
            //if (Plugin._serialPort[indexOfSelectedPedal_u].IsOpen)
            {



                // Create a Stopwatch instance
                Stopwatch stopwatch = new Stopwatch();

                // Start the stopwatch
                stopwatch.Start();



                SerialPort sp = Plugin._serialPort[pedalSelected];



                // https://stackoverflow.com/questions/9732709/the-calling-thread-cannot-access-this-object-because-a-different-thread-owns-it


                //int length = sizeof(DAP_config_st);




                if (sp.IsOpen)
                {
                    int receivedLength = sp.BytesToRead;

                    if (receivedLength > 0)
                    {


                        timeCntr[pedalSelected] += 1;


                        // determine byte sequence which is defined as message end --> crlf
                        byte[] byteToFind = System.Text.Encoding.GetEncoding(28591).GetBytes(STOPCHAR[0].ToCharArray());

                        // check if buffer is large enough otherwise discard in buffer and set offset to 0
                        if (bufferSize > (appendedBufferOffset[pedalSelected] + receivedLength))
                        {
                            sp.Read(buffer_appended[pedalSelected], appendedBufferOffset[pedalSelected], receivedLength);
                        }
                        else
                        {
                            sp.DiscardInBuffer();
                            appendedBufferOffset[pedalSelected] = 0;
                        }


                        // calculate current buffer length
                        int currentBufferLength = appendedBufferOffset[pedalSelected] + receivedLength;


                        // copy to local buffer
                        byte[] localBuffer = new byte[currentBufferLength];
                        Buffer.BlockCopy(buffer_appended[pedalSelected], 0, localBuffer, 0, currentBufferLength);


                        // find all occurences of crlf as they indicate message end
                        List<int> indices = FindAllOccurrences(localBuffer, byteToFind, currentBufferLength);




                        // Destination array
                        byte[] destinationArray = new byte[1000];

                        // copy the last not finished buffer element to begining of next cycles buffer
                        // and determine buffer offset
                        if (indices.Count > 0)
                        {
                            // If at least one crlf was detected, check whether it arrieved at the last bytes
                            int lastElement = indices.Last<int>();
                            int remainingMessageLength = currentBufferLength - (lastElement + 2);
                            if (remainingMessageLength >= 0)
                            {
                                appendedBufferOffset[pedalSelected] = remainingMessageLength;

                                Buffer.BlockCopy(buffer_appended[pedalSelected], lastElement + 2, buffer_appended[pedalSelected], 0, remainingMessageLength);
                            }
                        }
                        else
                        {
                            appendedBufferOffset[pedalSelected] += receivedLength;
                        }





                        int srcBufferOffset = 0;
                        // decode every message
                        foreach (int number in indices)
                        {
                            // computes the length of bytes to read
                            int destBuffLength = number - srcBufferOffset;
                            if (destBuffLength > 1000)
                            {
                                continue;
                            }


                            // copy bytes to subarray
                            Buffer.BlockCopy(localBuffer, srcBufferOffset, destinationArray, 0, destBuffLength);

                            // update src buffer offset
                            srcBufferOffset = number + 2;



                            // check for pedal state struct
                            if ((destBuffLength == sizeof(DAP_state_basic_st)))
                            {

                                // parse byte array as config struct
                                DAP_state_basic_st pedalState_read_st = getStateFromBytes(destinationArray);

                                // check whether receive struct is plausible
                                DAP_state_basic_st* v_state = &pedalState_read_st;
                                byte* p_state = (byte*)v_state;

                                // payload type check
                                bool check_payload_state_b = false;
                                if (pedalState_read_st.payloadHeader_.payloadType == Constants.pedalStateBasicPayload_type)
                                {
                                    check_payload_state_b = true;
                                }

                                // CRC check
                                bool check_crc_state_b = false;
                                if (Plugin.checksumCalc(p_state, sizeof(payloadHeader) + sizeof(payloadPedalState_Basic)) == pedalState_read_st.payloadFooter_.checkSum)
                                {
                                    check_crc_state_b = true;
                                }

                                if ((check_payload_state_b) && check_crc_state_b)
                                {

                                    // write vJoy data
                                    if (Plugin.Settings.vjoy_output_flag == 1)
                                    {
                                        switch (pedalSelected)
                                        {

                                            case 0:
                                                //joystick.SetJoystickAxis(pedalState_read_st.payloadPedalState_.joystickOutput_u16, Axis.HID_USAGE_RX);  // Center X axis
                                                joystick.SetAxis(pedalState_read_st.payloadPedalBasicState_.joystickOutput_u16, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_RX);   // HID_USAGES Enums
                                                break;
                                            case 1:
                                                //joystick.SetJoystickAxis(pedalState_read_st.payloadPedalState_.joystickOutput_u16, Axis.HID_USAGE_RY);  // Center X axis
                                                joystick.SetAxis(pedalState_read_st.payloadPedalBasicState_.joystickOutput_u16, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_RY);   // HID_USAGES Enums
                                                break;
                                            case 2:
                                                //joystick.SetJoystickAxis(pedalState_read_st.payloadPedalState_.joystickOutput_u16, Axis.HID_USAGE_RZ);  // Center X axis
                                                joystick.SetAxis(pedalState_read_st.payloadPedalBasicState_.joystickOutput_u16, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_RZ);   // HID_USAGES Enums
                                                break;
                                            default:
                                                break;
                                        }

                                    }



                                    // GUI update
                                    if ((pedalStateHasAlreadyBeenUpdated_b == false) && (indexOfSelectedPedal_u == pedalSelected))
                                    {
                                        //TextBox_debugOutput.Text = "Pedal pos: " + pedalState_read_st.payloadPedalState_.pedalPosition_u16;
                                        //TextBox_debugOutput.Text += "Pedal force: " + pedalState_read_st.payloadPedalState_.pedalForce_u16;
                                        //TextBox_debugOutput.Text += ",  Servo pos targe: " + pedalState_read_st.payloadPedalState_.servoPosition_i16;
                                        //TextBox_debugOutput.Text += ",  Servo pos: " + pedalState_read_st.payloadPedalState_.servoPosition_i16;



                                        pedalStateHasAlreadyBeenUpdated_b = true;

                                        text_point_pos.Visibility = Visibility.Hidden;
                                        double control_rect_value_max = 65535;
                                        double dyy = canvas.Height / control_rect_value_max;
                                        double dxx = canvas.Width / control_rect_value_max;

                                        if (debug_flag)
                                        {
                                            Canvas.SetLeft(rect_State, dxx * pedalState_read_st.payloadPedalBasicState_.pedalPosition_u16 - rect_State.Width / 2);
                                            Canvas.SetTop(rect_State, canvas.Height - dyy * pedalState_read_st.payloadPedalBasicState_.pedalForce_u16 - rect_State.Height / 2);

                                            Canvas.SetLeft(text_state, Canvas.GetLeft(rect_State) /*+ rect_State.Width*/);
                                            Canvas.SetTop(text_state, Canvas.GetTop(rect_State) - rect_State.Height);
                                            text_state.Text = Math.Round(pedalState_read_st.payloadPedalBasicState_.pedalForce_u16 / control_rect_value_max * 100) + "%";

                                        }
                                        else
                                        {
                                            Canvas.SetLeft(rect_State, dxx * pedalState_read_st.payloadPedalBasicState_.pedalPosition_u16 - rect_State.Width / 2);
                                            int round_x = (int)(100 * pedalState_read_st.payloadPedalBasicState_.pedalPosition_u16 / control_rect_value_max) - 1;
                                            int x_showed = round_x + 1;
                                            round_x = Math.Max(0, Math.Min(round_x, 99));
                                            Canvas.SetTop(rect_State, canvas.Height - Force_curve_Y[round_x] - rect_State.Height / 2);
                                            Canvas.SetLeft(text_state, Canvas.GetLeft(rect_State) /*+ rect_State.Width*/);
                                            Canvas.SetTop(text_state, Canvas.GetTop(rect_State) - rect_State.Height);
                                            text_state.Text = x_showed + "%";
                                        }

                                    }


                                    continue;
                                }

                            }






                            // check for pedal extended state struct
                            if ((destBuffLength == sizeof(DAP_state_extended_st)))
                            {

                                // parse byte array as config struct
                                DAP_state_extended_st pedalState_ext_read_st = getStateExtFromBytes(destinationArray);

                                // check whether receive struct is plausible
                                DAP_state_extended_st* v_state = &pedalState_ext_read_st;
                                byte* p_state = (byte*)v_state;

                                // payload type check
                                bool check_payload_state_b = false;
                                if (pedalState_ext_read_st.payloadHeader_.payloadType == Constants.pedalStateExtendedPayload_type)
                                {
                                    check_payload_state_b = true;
                                }

                                // CRC check
                                bool check_crc_state_b = false;
                                if (Plugin.checksumCalc(p_state, sizeof(payloadHeader) + sizeof(payloadPedalState_Extended)) == pedalState_ext_read_st.payloadFooter_.checkSum)
                                {
                                    check_crc_state_b = true;
                                }

                                if ((check_payload_state_b) && check_crc_state_b)
                                {




                                    if (indexOfSelectedPedal_u == pedalSelected)
                                    {
                                        if (dumpPedalToResponseFile[indexOfSelectedPedal_u])
                                        {
                                            // Specify the path to the file
                                            string currentDirectory = Directory.GetCurrentDirectory();
                                            string filePath = currentDirectory + "\\PluginsData\\Common" + "\\output_" + indexOfSelectedPedal_u.ToString() + ".txt";


                                            // write header
                                            if (!File.Exists(filePath))
                                            {
                                                using (StreamWriter writer = new StreamWriter(filePath, true))
                                                {
                                                    // Write the content to the file
                                                    writer.Write("cycleCtr, ");
                                                    writer.Write("time_InMs, ");
                                                    writer.Write("forceRaw_InKg, ");
                                                    writer.Write("forceFiltered_InKg, ");
                                                    writer.Write("forceVelocity_InKgPerSec, ");
                                                    writer.Write("servoPos_InSteps, ");
                                                    writer.Write("servoPosEsp_InSteps, ");
                                                    writer.Write("servoCurrent_InPercent, ");
                                                    writer.Write("servoVoltage_InV");
                                                    writer.Write("\n");
                                                }

                                            }
                                            // Use StreamWriter to write to the file
                                            using (StreamWriter writer = new StreamWriter(filePath, true))
                                            {
                                                // Write the content to the file
                                                writeCntr++;
                                                writer.Write(writeCntr);
                                                writer.Write(", ");
                                                writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.timeInMs_u32);
                                                writer.Write(", ");
                                                writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.pedalForce_raw_fl32);
                                                writer.Write(", ");
                                                writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.pedalForce_filtered_fl32);
                                                writer.Write(", ");
                                                writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.forceVel_est_fl32);
                                                writer.Write(", ");
                                                writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.servoPosition_i16);
                                                writer.Write(", ");
                                                writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.servoPositionTarget_i16);
                                                writer.Write(", ");
                                                writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.servo_current_percent_i16);
                                                writer.Write(", ");
                                                writer.Write(((float)pedalState_ext_read_st.payloadPedalExtendedState_.servo_voltage_0p1V_i16) / 10.0);
                                                writer.Write("\n");
                                            }
                                        }
                                    }




                                    continue;
                                }
                            }








                            // decode into config struct
                            if ((waiting_for_pedal_config[pedalSelected]) && (destBuffLength == sizeof(DAP_config_st)))
                            {

                                // parse byte array as config struct
                                DAP_config_st pedalConfig_read_st = getConfigFromBytes(destinationArray);

                                // check whether receive struct is plausible
                                DAP_config_st* v_config = &pedalConfig_read_st;
                                byte* p_config = (byte*)v_config;

                                // payload type check
                                bool check_payload_config_b = false;
                                if (pedalConfig_read_st.payloadHeader_.payloadType == Constants.pedalConfigPayload_type)
                                {
                                    check_payload_config_b = true;
                                }

                                // CRC check
                                bool check_crc_config_b = false;
                                if (Plugin.checksumCalc(p_config, sizeof(payloadHeader) + sizeof(payloadPedalConfig)) == pedalConfig_read_st.payloadFooter_.checkSum)
                                {
                                    check_crc_config_b = true;
                                }

                                if ((check_payload_config_b) && check_crc_config_b)
                                {
                                    waiting_for_pedal_config[pedalSelected] = false;
                                    dap_config_st[pedalSelected] = pedalConfig_read_st;
                                    updateTheGuiFromConfig();

                                    continue;
                                }
                                else
                                {
                                    TextBox_debugOutput.Text = "Payload config test 1: " + check_payload_config_b;
                                    TextBox_debugOutput.Text += "Payload config test 2: " + check_crc_config_b;
                                }

                            }


                            // If non known array datatype was received, assume a text message was received and print it
                            byte[] destinationArray_sub = new byte[destBuffLength];
                            Buffer.BlockCopy(destinationArray, 0, destinationArray_sub, 0, destBuffLength);
                            string resultString = Encoding.GetEncoding(28591).GetString(destinationArray_sub);

                            TextBox_serialMonitor.Text += resultString + "\n";
                            TextBox_serialMonitor.ScrollToEnd();




                            // When only a few messages are received, make the counter greater than N thus every message is printed
                            //if (destBuffLength < 100)
                            //{
                            //    printCtr = 600;
                            //}

                            //if (printCtr++ > 200)
                            //{
                            //    printCtr = 0;
                            //    TextBox_serialMonitor.Text += dataToSend + "\n";
                            //    TextBox_serialMonitor.ScrollToEnd();
                            //}





                        }






                        // Stop the stopwatch
                        stopwatch.Stop();

                        // Get the elapsed time
                        TimeSpan elapsedTime = stopwatch.Elapsed;

                        timeCollector[pedalSelected] += elapsedTime.TotalMilliseconds;

                        if (timeCntr[pedalSelected] >= 50)
                        {


                            double avgTime = timeCollector[pedalSelected] / timeCntr[pedalSelected];
                            TextBox_debugOutput.Text = "Serial callback time in ms: " + avgTime.ToString();

                            timeCntr[pedalSelected] = 0;
                            timeCollector[pedalSelected] = 0;
                        }
                    }

                }
            }
        }



        /********************************************************************************************************************/
        /*							Connect to pedal																		*/
        /********************************************************************************************************************/


        unsafe public void ConnectToPedal_click(object sender, RoutedEventArgs e)
        {

            Plugin.Settings.connect_flag[indexOfSelectedPedal_u] = 1;
            if (ConnectToPedal.IsChecked == false)
            {
                if (Plugin._serialPort[indexOfSelectedPedal_u].IsOpen == false)
                {
                    try
                    {
                        openSerialAndAddReadCallback(indexOfSelectedPedal_u);
                        TextBox_debugOutput.Text = "Serialport open";
                        ConnectToPedal.IsChecked = true;
                        btn_pedal_connect.Content = "Disconnect From Pedal";

                        // register a callback that is triggered when serial data is received
                        // see https://gist.github.com/mini-emmy/9617732
                        //Plugin._serialPort[indexOfSelectedPedal_u].DataReceived += new SerialDataReceivedEventHandler(sp_DataReceived);

                        System.Threading.Thread.Sleep(100);

                        Plugin.Settings.connect_status[indexOfSelectedPedal_u] = 1;

                    }
                    catch (Exception ex)
                    {
                        TextBox_debugOutput.Text = ex.Message;
                        ConnectToPedal.IsChecked = false;
                    }

                }
                else
                {
                    closeSerialAndStopReadCallback(indexOfSelectedPedal_u);

                    //Plugin._serialPort[indexOfSelectedPedal_u].DataReceived -= sp_DataReceived;

                    ConnectToPedal.IsChecked = false;
                    TextBox_debugOutput.Text = "Serialport already open, close it";
                    Plugin.Settings.connect_status[indexOfSelectedPedal_u] = 0;
                    Plugin.Settings.connect_flag[indexOfSelectedPedal_u] = 0;
                    Plugin.connectSerialPort[indexOfSelectedPedal_u] = false;
                    btn_pedal_connect.Content = "Connect To Pedal";
                }
            }
            else
            {
                ConnectToPedal.IsChecked = false;
                closeSerialAndStopReadCallback(indexOfSelectedPedal_u);
                TextBox_debugOutput.Text = "Serialport close";
                Plugin.connectSerialPort[indexOfSelectedPedal_u] = false;
                Plugin.Settings.connect_status[indexOfSelectedPedal_u] = 0;
                Plugin.Settings.connect_flag[indexOfSelectedPedal_u] = 0;
                btn_pedal_connect.Content = "Connect To Pedal";

            }

            ////reading config from pedal

            if (checkbox_pedal_read.IsChecked == true)
            {
                Reading_config_auto(indexOfSelectedPedal_u);
            }
            updateTheGuiFromConfig();
        }

        /********************************************************************************************************************/
        /*							Serial port selection																	*/
        /********************************************************************************************************************/
        public void UpdateSerialPortList_click(object sender, RoutedEventArgs e)
        {
            UpdateSerialPortList_click();
        }

        public void SerialPortSelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            string tmp = (string)SerialPortSelection.SelectedValue;
            //Plugin._serialPort[indexOfSelectedPedal_u].PortName = tmp;


            //try 
            //{
            //    TextBox_debugOutput.Text = "Debug: " + Plugin.Settings.selectedComPortNames[indexOfSelectedPedal_u];
            //}
            //catch (Exception caughtEx)
            //{
            //    string errorMessage = caughtEx.Message;
            //    TextBox_debugOutput.Text = errorMessage;
            //}

            try
            {
                //if (Plugin.Settings.connect_status[indexOfSelectedPedal_u] == 0)
                if (Plugin._serialPort[indexOfSelectedPedal_u].IsOpen == false)
                {
                    Plugin.Settings.selectedComPortNames[indexOfSelectedPedal_u] = tmp;
                    Plugin._serialPort[indexOfSelectedPedal_u].PortName = tmp;
                }
                TextBox_debugOutput.Text = "COM port selected: " + Plugin.Settings.selectedComPortNames[indexOfSelectedPedal_u];

            }
            catch (Exception caughtEx)
            {
                string errorMessage = caughtEx.Message;
                TextBox_debugOutput.Text = errorMessage;
            }



        }





        public void AbsPatternChanged(object sender, SelectionChangedEventArgs e)
        {
            try
            {
                dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.absPattern = (byte)AbsPattern.SelectedIndex;
            }
            catch (Exception caughtEx)
            {
                string errorMessage = caughtEx.Message;
                TextBox_debugOutput.Text = errorMessage;
            }
        }




        public void SpindlePitchChanged(object sender, SelectionChangedEventArgs e)
        {
            try
            {
                dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.spindlePitch_mmPerRev_u8 = (byte)SpindlePitch.SelectedIndex;
            }
            catch (Exception caughtEx)
            {
                string errorMessage = caughtEx.Message;
                TextBox_debugOutput.Text = errorMessage;
            }
        }








        private void RestartPedal_click(object sender, RoutedEventArgs e)
        {
            Plugin._serialPort[indexOfSelectedPedal_u].DtrEnable = true;
            Plugin._serialPort[indexOfSelectedPedal_u].RtsEnable = true;
            System.Threading.Thread.Sleep(100);
            Plugin._serialPort[indexOfSelectedPedal_u].DtrEnable = false;
            Plugin._serialPort[indexOfSelectedPedal_u].RtsEnable = false;
        }

        public void Read_for_slot(object sender, EventArgs e)
        {
            var Button = sender as SHButtonPrimary;
            
            using (System.Windows.Forms.OpenFileDialog openFileDialog = new System.Windows.Forms.OpenFileDialog())
            {
                openFileDialog.Title = "Datei auswählen";
                openFileDialog.Filter = "Configdateien (*.json)|*.json";
                string currentDirectory = Directory.GetCurrentDirectory();
                openFileDialog.InitialDirectory = currentDirectory + "\\PluginsData\\Common";

                if (openFileDialog.ShowDialog() == DialogResult.OK)
                {
                    string content = (string)openFileDialog.FileName;


                    string filePath = openFileDialog.FileName;
                    //TextBox_debugOutput.Text =  Button.Name; 
                    if (Button.Name == "Reading_clutch")
                    {

                        Plugin.Settings.pedal_file_string[profile_select,0] = filePath;
                        Label_clutch_file.Content = Plugin.Settings.pedal_file_string[profile_select, 0];
                        Plugin.Settings.file_enable[profile_select, 0] = 1;
                        Clutch_file_check.IsChecked = true;

                    }
                    if (Button.Name == "Reading_brake")
                    {
                        Plugin.Settings.pedal_file_string[profile_select, 1] = filePath;
                        Label_brake_file.Content = Plugin.Settings.pedal_file_string[profile_select, 1];
                        Plugin.Settings.file_enable[profile_select, 1] = 1;
                        Brake_file_check.IsChecked = true;
                    }
                    if (Button.Name == "Reading_gas")
                    {
                        Plugin.Settings.pedal_file_string[profile_select, 2] = filePath;
                        Label_gas_file.Content = Plugin.Settings.pedal_file_string[profile_select, 2];
                        Plugin.Settings.file_enable[profile_select, 2] = 1;
                        Gas_file_check.IsChecked = true;
                    }


                }
            }
        }

        public void Clear_slot(object sender, EventArgs e)
        {
            var Button = sender as SHButtonPrimary;
            if (Button.Name == "Clear_clutch")
            {

                Plugin.Settings.pedal_file_string[profile_select, 0] = "";
                Label_clutch_file.Content = Plugin.Settings.pedal_file_string[profile_select, 0];
                Plugin.Settings.file_enable[profile_select, 0] = 0;
                Clutch_file_check.IsChecked = false;

            }
            if (Button.Name == "Clear_brake")
            {

                Plugin.Settings.pedal_file_string[profile_select, 1] = "";
                Label_brake_file.Content = Plugin.Settings.pedal_file_string[profile_select, 1];
                Plugin.Settings.file_enable[profile_select, 1] = 0;
                Brake_file_check.IsChecked = false;

            }
            if (Button.Name == "Clear_gas")
            {

                Plugin.Settings.pedal_file_string[profile_select, 2] = "";
                Label_gas_file.Content = Plugin.Settings.pedal_file_string[profile_select, 2];
                Plugin.Settings.file_enable[profile_select, 2] = 0;
                Gas_file_check.IsChecked = false;

            }
            //updateTheGuiFromConfig();
        }
        void Parsefile(uint profile_index)
        {
            // https://learn.microsoft.com/en-us/dotnet/standard/serialization/system-text-json/deserialization


            // c# code to iterate over all fields of struct and set values from json file
            for (uint pedalIdx = 0; pedalIdx < 3; pedalIdx++)
            {
                if (Plugin.Settings.file_enable[profile_select, pedalIdx] == 1)
                {
                    payloadPedalConfig payloadPedalConfig_fromJson_st = dap_config_st[pedalIdx].payloadPedalConfig_;
                    // Read the entire JSON file
                    string jsonString = File.ReadAllText(Plugin.Settings.pedal_file_string[profile_index, pedalIdx]);
                    // Parse all of the JSON.
                    //JsonNode forecastNode = JsonNode.Parse(jsonString);
                    dynamic data = JsonConvert.DeserializeObject(jsonString);
                    //var s = default(payloadPedalConfig);
                    Object obj = payloadPedalConfig_fromJson_st;// s;
                    FieldInfo[] fi = payloadPedalConfig_fromJson_st.GetType().GetFields(BindingFlags.Public | BindingFlags.Instance);
                    // Iterate over each field and print its name and value
                    foreach (var field in fi)
                    {

                        if (data["payloadPedalConfig_"][field.Name] != null)
                        //if (forecastNode["payloadPedalConfig_"][field.Name] != null)
                        {
                            try
                            {
                                if (field.FieldType == typeof(float))
                                {
                                    //float value = forecastNode["payloadPedalConfig_"][field.Name].GetValue<float>();
                                    float value = (float)data["payloadPedalConfig_"][field.Name];
                                    field.SetValue(obj, value);
                                }

                                if (field.FieldType == typeof(byte))
                                {
                                    //byte value = forecastNode["payloadPedalConfig_"][field.Name].GetValue<byte>();
                                    byte value = (byte)data["payloadPedalConfig_"][field.Name];
                                    field.SetValue(obj, value);
                                }

                            }
                            catch (Exception)
                            {

                            }

                        }
                    }

                    // set values in global structure
                    dap_config_st[pedalIdx].payloadPedalConfig_ = (payloadPedalConfig)obj;// payloadPedalConfig_fromJson_st;
                }
                
            }
            


            updateTheGuiFromConfig();
        }
    

        

        
        private void OpenButton_Click(object sender, EventArgs e)
        {
            using (System.Windows.Forms.OpenFileDialog openFileDialog = new System.Windows.Forms.OpenFileDialog())
            {
                openFileDialog.Title = "Datei auswählen";
                openFileDialog.Filter = "Configdateien (*.json)|*.json";
                string currentDirectory = Directory.GetCurrentDirectory();
                openFileDialog.InitialDirectory = currentDirectory + "\\PluginsData\\Common";

                if (openFileDialog.ShowDialog() == DialogResult.OK)
                {
                    string content = (string)openFileDialog.FileName;
                    TextBox_debugOutput.Text = content;

                    string filePath = openFileDialog.FileName;


                    if (false)
                    {
                        string text1 = System.IO.File.ReadAllText(filePath);
                        DataContractJsonSerializer deserializer = new DataContractJsonSerializer(typeof(DAP_config_st));
                        var ms = new MemoryStream(Encoding.UTF8.GetBytes(text1));
                        dap_config_st[indexOfSelectedPedal_u] = (DAP_config_st)deserializer.ReadObject(ms);
                    }
                    else
                    {
                        // https://learn.microsoft.com/en-us/dotnet/standard/serialization/system-text-json/deserialization


                        // c# code to iterate over all fields of struct and set values from json file

                        // Read the entire JSON file
                        string jsonString = File.ReadAllText(filePath);

                        // Parse all of the JSON.
                        //JsonNode forecastNode = JsonNode.Parse(jsonString);
                        dynamic data = JsonConvert.DeserializeObject(jsonString);



                        payloadPedalConfig payloadPedalConfig_fromJson_st = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_;
                        //var s = default(payloadPedalConfig);
                        Object obj = payloadPedalConfig_fromJson_st;// s;



                        FieldInfo[] fi = payloadPedalConfig_fromJson_st.GetType().GetFields(BindingFlags.Public | BindingFlags.Instance);

                        // Iterate over each field and print its name and value
                        foreach (var field in fi)
                        {

                            if (data["payloadPedalConfig_"][field.Name] != null)
                            //if (forecastNode["payloadPedalConfig_"][field.Name] != null)
                            {
                                try
                                {
                                    if (field.FieldType == typeof(float))
                                    {
                                        //float value = forecastNode["payloadPedalConfig_"][field.Name].GetValue<float>();
                                        float value = (float)data["payloadPedalConfig_"][field.Name];
                                        field.SetValue(obj, value);
                                    }

                                    if (field.FieldType == typeof(byte))
                                    {
                                        //byte value = forecastNode["payloadPedalConfig_"][field.Name].GetValue<byte>();
                                        byte value = (byte)data["payloadPedalConfig_"][field.Name];
                                        field.SetValue(obj, value);
                                    }


                                }
                                catch (Exception)
                                {

                                }

                            }
                        }

                        // set values in global structure
                        dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_ = (payloadPedalConfig)obj;// payloadPedalConfig_fromJson_st;
                    }

                    updateTheGuiFromConfig();
                    TextBox_debugOutput.Text = "Config new imported!";
                    TextBox2.Text = "Open " + openFileDialog.FileName;
                }
            }

        }

        private void SaveButton_Click(object sender, RoutedEventArgs e)
        {
            using (System.Windows.Forms.SaveFileDialog saveFileDialog = new System.Windows.Forms.SaveFileDialog())
            {
                saveFileDialog.Title = "Datei speichern";
                saveFileDialog.Filter = "Textdateien (*.json)|*.json";
                string currentDirectory = Directory.GetCurrentDirectory();
                saveFileDialog.InitialDirectory = currentDirectory + "\\PluginsData\\Common";

                if (saveFileDialog.ShowDialog() == DialogResult.OK)
                {
                     string fileName = saveFileDialog.FileName;


                    this.dap_config_st[indexOfSelectedPedal_u].payloadHeader_.version = (byte)Constants.pedalConfigPayload_version;

                    // https://stackoverflow.com/questions/3275863/does-net-4-have-a-built-in-json-serializer-deserializer
                    // https://learn.microsoft.com/en-us/dotnet/framework/wcf/feature-details/how-to-serialize-and-deserialize-json-data?redirectedfrom=MSDN
                    var stream1 = new MemoryStream();
                    //var ser = new DataContractJsonSerializer(typeof(DAP_config_st));
                    //ser.WriteObject(stream1, dap_config_st[indexOfSelectedPedal_u]);


                    // formatted JSON see https://stackoverflow.com/a/38538454
                    var writer = JsonReaderWriterFactory.CreateJsonWriter(stream1, Encoding.UTF8, true, true, "  ");
                    var serializer = new DataContractJsonSerializer(typeof(DAP_config_st));
                    serializer.WriteObject(writer, dap_config_st[indexOfSelectedPedal_u]);
                    writer.Flush();

                    stream1.Position = 0;
                    StreamReader sr = new StreamReader(stream1);
                    string jsonString = sr.ReadToEnd();

                    // Check if file already exists. If yes, delete it.     
                    if (File.Exists(fileName))
                    {
                        File.Delete(fileName);
                    }


                    System.IO.File.WriteAllText(fileName, jsonString);
                    TextBox_debugOutput.Text = "Config new exported!";
                    TextBox2.Text = "Save " + saveFileDialog.FileName;
                    }
            }
        }

        
        private void DisconnectToPedal_click(object sender, RoutedEventArgs e)
        {

            closeSerialAndStopReadCallback(indexOfSelectedPedal_u);
            Plugin.Settings.connect_flag[indexOfSelectedPedal_u] = 0;

            if (ConnectToPedal.IsChecked == true)
            {
                ConnectToPedal.IsChecked = false;
                TextBox_debugOutput.Text = "Serialport close";
                Plugin.Settings.connect_status[indexOfSelectedPedal_u] = 0;
            }           
            else
            {
                ConnectToPedal.IsChecked = false;
                TextBox_debugOutput.Text = "Not Checked Serialport close";
            }
            updateTheGuiFromConfig();

        }

        private void dump_pedal_response_to_file_checked(object sender, RoutedEventArgs e)
        {
            dumpPedalToResponseFile[indexOfSelectedPedal_u] = true;
        }

        private void dump_pedal_response_to_file_unchecked(object sender, RoutedEventArgs e)
        {
            dumpPedalToResponseFile[indexOfSelectedPedal_u] = false;
        }



        private void Simulate_ABS_check_Checked(object sender, RoutedEventArgs e)
        {
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.Simulate_ABS_trigger = 1;
            TextBox_debugOutput.Text = "simulateABS: on";
            rect_SABS.Visibility = Visibility.Visible;
            rect_SABS_Control.Visibility = Visibility.Visible;
            text_SABS.Visibility = Visibility.Visible;

        }
        private void Simulate_ABS_check_Unchecked(object sender, RoutedEventArgs e)
        {
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.Simulate_ABS_trigger = 0;
            TextBox_debugOutput.Text = "simulateABS: off";
            rect_SABS.Visibility = Visibility.Hidden;
            rect_SABS_Control.Visibility = Visibility.Hidden;
            text_SABS.Visibility = Visibility.Hidden;

        }



        //dragable control rect.

        /*private void InitializeRectanglePositions()
        {
            rectanglePositions.Add("rect1", new Point(75, 75));
            rectanglePositions.Add("rect2", new Point(155, 55));
            rectanglePositions.Add("rect3", new Point(235, 35));
            rectanglePositions.Add("rect4", new Point(315, 15));
        }*/

        private void Rectangle_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            isDragging = true;
            var rectangle = sender as Rectangle;
            offset = e.GetPosition(rectangle);
            rectangle.CaptureMouse();
            //SolidColorBrush buttonBackground = btn_update.Background as SolidColorBrush;
            //Color light = Color.FromArgb(255, buttonBackground.Color.R, buttonBackground.Color.G, buttonBackground.Color.B);
            //Color middle_light= Color.FromArgb(235, buttonBackground.Color.R, buttonBackground.Color.G, buttonBackground.Color.B);
            //Color dark = Color.FromArgb(100, buttonBackground.Color.R, buttonBackground.Color.G, buttonBackground.Color.B);
            //
            /*
            RadialGradientBrush myRadialGradientBrush = new RadialGradientBrush();
            myRadialGradientBrush.GradientOrigin = new Point(0.5, 0.5);
            myRadialGradientBrush.Center = new Point(0.5, 0.5);
            myRadialGradientBrush.RadiusX = 0.5;
            myRadialGradientBrush.RadiusY = 0.5;
            myRadialGradientBrush.GradientStops.Add(
                new GradientStop(light, 0.0));
            myRadialGradientBrush.GradientStops.Add(
                new GradientStop(middle_light, 0.75));
            myRadialGradientBrush.GradientStops.Add(
                new GradientStop(dark, 1.0));
            */
            //
            //rectangle.Fill = new SolidColorBrush(light);
            //rectangle.Fill = myRadialGradientBrush;
            //Color light = Color.FromArgb(128, 128, 128, 128);
            if(rectangle.Name != "rect_SABS_Control" & rectangle.Name != "rect_BP_Control")
            {
                var dropShadowEffect = new DropShadowEffect
                {
                    ShadowDepth = 0,
                    BlurRadius = 15,
                    Color = Colors.White,
                    Opacity = 1
                };
                rectangle.Fill = lightcolor;
                rectangle.Effect = dropShadowEffect;
            }

        }

        private void Rectangle_MouseMove(object sender, MouseEventArgs e)
        {
            if (isDragging)
            {
                var rectangle = sender as Rectangle;
                //double x = e.GetPosition(canvas).X - offset.X;
                double y = e.GetPosition(canvas).Y - offset.Y;

                // Ensure the rectangle stays within the canvas
                //x = Math.Max(0, Math.Min(x, canvas.ActualWidth - rectangle.ActualWidth));
                y = Math.Max(-1*rectangle.Height/2, Math.Min(y, canvas.Height - rectangle.Height/2));

                //Canvas.SetLeft(rectangle, x);
                Canvas.SetTop(rectangle, y);
                double y_max = 100;
                double dx = canvas.Height / y_max;
                double y_actual = (canvas.Height - y -rectangle.Height/2)/dx;
                if (rectangle.Name == "rect0")
                {
                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.relativeForce_p000 = Convert.ToByte(y_actual);
                    text_point_pos.Text = "Travel:0%";
                    text_point_pos.Text += "\nForce: "+(int)y_actual+"%";
                    
                }
                if (rectangle.Name == "rect1")
                {

                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.relativeForce_p020 = Convert.ToByte(y_actual);
                    text_point_pos.Text = "Travel:20%";
                    text_point_pos.Text += "\nForce: " + (int)y_actual + "%";
                }
                if (rectangle.Name == "rect2")
                {
                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.relativeForce_p040 = Convert.ToByte(y_actual);
                    text_point_pos.Text = "Travel:40%";
                    text_point_pos.Text += "\nForce: " + (int)y_actual + "%";
                }
                if (rectangle.Name == "rect3")
                {
                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.relativeForce_p060 = Convert.ToByte(y_actual);
                    text_point_pos.Text = "Travel:60%";
                    text_point_pos.Text += "\nForce: " + (int)y_actual + "%";
                }
                if (rectangle.Name == "rect4")
                {
                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.relativeForce_p080 = Convert.ToByte(y_actual);
                    text_point_pos.Text = "Travel:80%";
                    text_point_pos.Text += "\nForce: " + (int)y_actual + "%";
                }
                if (rectangle.Name == "rect5")
                {
                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.relativeForce_p100 = Convert.ToByte(y_actual);
                    text_point_pos.Text = "Travel:100%";
                    text_point_pos.Text += "\nForce: " + (int)y_actual + "%";
                }
                text_point_pos.Visibility = Visibility.Visible; ;

                Update_BrakeForceCurve();



                // Update the position in the dictionary
                //rectanglePositions[rectangle.Name] = new Point(x, y);
            }
        }

        private void Rectangle_MouseMove_H(object sender, MouseEventArgs e)
        {
            if (isDragging)
            {
                var rectangle = sender as Rectangle;
                double x = e.GetPosition(canvas_horz_slider).X - offset.X;
                //double y = e.GetPosition(canvas).Y - offset.Y;

                // Ensure the rectangle stays within the canvas

                double min_posiiton = Canvas.GetLeft(rect6) + rectangle.Width / 2;
                double max_position = Canvas.GetLeft(rect7) - rectangle.Width / 2;
                double min_pedal_position = (canvas_horz_slider.Width - 10) * 0.05 + rect6.Width;
                double max_pedal_position = (canvas_horz_slider.Width - 10) * 0.95 + rect7.Width;
                double dx = 100 / (canvas_horz_slider.Width - 10);
                if (rectangle.Name == "rect6")
                {
                    x = Math.Max(min_pedal_position - 1 * rectangle.Width / 2, Math.Min(x, max_position));
                    double actual_x = (x - 5) * dx;
                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalStartPosition = Convert.ToByte(actual_x);

                    if (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalStartPosition > dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalEndPosition)
                    {
                        dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalStartPosition = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalEndPosition;
                        //PedalMinPos_Slider.Value = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalStartPosition;
                    }
                    TextBox_debugOutput.Text = "Pedal min position:" + dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalStartPosition;
                    Canvas.SetLeft(text_min_pos, x - text_min_pos.Width / 2 + rect6.Width / 2);
                    Canvas.SetTop(text_min_pos, 5);
                }
                if (rectangle.Name == "rect7")
                {
                    x = Math.Max(min_posiiton, Math.Min(x, max_pedal_position - rectangle.Width / 2));
                    double actual_x = (x - 5) * dx;
                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalEndPosition = Convert.ToByte(actual_x);

                    if (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalEndPosition < dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalStartPosition)
                    {
                        dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalEndPosition = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalStartPosition;
                        //PedalMaxPos_Slider.Value = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalEndPosition;
                    }
                    TextBox_debugOutput.Text = "Pedal max position:" + dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalEndPosition;
                    Canvas.SetLeft(text_max_pos, x - text_max_pos.Width / 2 + rect7.Width / 2);
                    Canvas.SetTop(text_max_pos, 5);
                }
                text_min_pos.Text = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalStartPosition + "%";
                text_max_pos.Text = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalEndPosition + "%";

                //y = Math.Max(-1 * rectangle.ActualHeight / 2, Math.Min(y, canvas.ActualHeight - rectangle.ActualHeight / 2));

                Canvas.SetLeft(rectangle, x);

            }
        }
        private void Rectangle_MouseMove_H_RPM(object sender, MouseEventArgs e)
        {
            if (isDragging)
            {
                var rectangle = sender as Rectangle;
                double x = e.GetPosition(canvas_horz_RPM_freq).X - offset.X;
                //double y = e.GetPosition(canvas).Y - offset.Y;

                // Ensure the rectangle stays within the canvas

                double min_posiiton = Canvas.GetLeft(rect_RPM_min) + rectangle.ActualWidth / 2;
                double max_position = Canvas.GetLeft(rect_RPM_max) - rectangle.ActualWidth / 2;
                double dx = 50 / (canvas_horz_RPM_freq.Width - 10);
                if (rectangle.Name == "rect_RPM_min")
                {
                    x = Math.Max(-1 * rectangle.ActualWidth / 2, Math.Min(x, max_position));
                    double actual_x = (x - 5) * dx;
                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.RPM_min_freq = Convert.ToByte(actual_x);
                    //TextBox_debugOutput.Text = "Pedal min position:" + dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalStartPosition;
                    Canvas.SetLeft(text_RPM_freq_min, Canvas.GetLeft(rect_RPM_min) - text_RPM_freq_min.Width / 2+rect_RPM_min.Width/2);
                    Canvas.SetTop(text_RPM_freq_min, 5);
                }
                if (rectangle.Name == "rect_RPM_max")
                {
                    x = Math.Max(min_posiiton, Math.Min(x, canvas_horz_slider.ActualWidth - rectangle.ActualWidth));
                    double actual_x = (x - 5) * dx;
                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.RPM_max_freq = Convert.ToByte(actual_x);

                    //TextBox_debugOutput.Text = "Pedal max position:" + dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalEndPosition;
                    Canvas.SetLeft(text_RPM_freq_max, Canvas.GetLeft(rect_RPM_max) - text_RPM_freq_max.Width / 2+rect_RPM_max.Width/2);
                    Canvas.SetTop(text_RPM_freq_max, 5);
                }
                text_RPM_freq_min.Text = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.RPM_min_freq + "Hz";
                text_RPM_freq_max.Text = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.RPM_max_freq + "Hz";

                //y = Math.Max(-1 * rectangle.ActualHeight / 2, Math.Min(y, canvas.ActualHeight - rectangle.ActualHeight / 2));

                Canvas.SetLeft(rectangle, x);

            }
        }
        private void Rectangle_MouseMove_V(object sender, MouseEventArgs e)
        {
            if (isDragging)
            {
                var rectangle = sender as Rectangle;
                double y = e.GetPosition(canvas_vert_slider).Y - offset.Y;
                //double y = e.GetPosition(canvas).Y - offset.Y;

                // Ensure the rectangle stays within the canvas

                double min_position =  Canvas.GetTop(rect8) - rectangle.Height / 2;
                double max_position = Canvas.GetTop(rect9) + rectangle.Height / 2;
                double dy = 250 / (canvas_vert_slider.Height);
                if (rectangle.Name == "rect8")
                {
                    y = Math.Max(max_position, Math.Min(y, canvas_vert_slider.Height + rectangle.Height / 2));
                    
                    double actual_y = (canvas_vert_slider.Height- y-rectangle.Height/2)  * dy;
                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.preloadForce = Convert.ToByte(actual_y);

                    if (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.preloadForce > dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.maxForce)
                    {
                        dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.preloadForce = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.maxForce;
                        //PedalMinForce_Slider.Value = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.preloadForce;
                    }
                    
                    //TextBox_debugOutput.Text = "Pedal min position:" + dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalStartPosition;
                    Canvas.SetLeft(text_min_force, rect8.Width+3);
                    Canvas.SetTop(text_min_force, Canvas.GetTop(rect8));
                }
                if (rectangle.Name == "rect9")
                {
                    y = Math.Max(-1 * rectangle.Height / 2, Math.Min(y, min_position ));
                    
                    double actual_y = (canvas_vert_slider.Height - y - rectangle.Height / 2) * dy;
                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.maxForce = Convert.ToByte(actual_y);
                    if (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.maxForce < dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.preloadForce)
                    {
                        dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.maxForce = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.preloadForce;
                        //PedalMaxForce_Slider.Value = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.maxForce;
                    }
                    
                    //TextBox_debugOutput.Text = "Pedal max position:" + dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalEndPosition;
                    Canvas.SetLeft(text_max_force,  rect9.Width+3);
                    Canvas.SetTop(text_max_force, Canvas.GetTop(rect9) - 6-text_max_force.Height/2);
                    
                    
                }
                text_min_force.Text = "Preload:\n" + dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.preloadForce + "kg";
                text_max_force.Text = "Max Force:\n" + dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.maxForce + "kg";
                
                //y = Math.Max(-1 * rectangle.ActualHeight / 2, Math.Min(y, canvas.ActualHeight - rectangle.ActualHeight / 2));

                Canvas.SetTop(rectangle, y);

            }
        }

        private void Rectangle_MouseMove_ABS(object sender, MouseEventArgs e)
        {
            if (isDragging)
            {
                var rectangle = sender as Rectangle;
                //double x = e.GetPosition(canvas).X - offset.X;
                double y = e.GetPosition(canvas).Y - offset.Y;

                // Ensure the rectangle stays within the canvas
                double dy = canvas.Height / 100;
                double min_posiiton = 5 * dy;
                double max_position = 50 * dy;
                //min position: 50%, max 95%
                //double dx = 100 / (canvas_horz_slider.Width - 10);
                y = Math.Max(min_posiiton, Math.Min(y, max_position));
                //Canvas.SetTop(rect_SABS, y);
                rect_SABS.Height = y;
                double actual_y = (canvas.Height -y)/dy;
                dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.Simulate_ABS_value = Convert.ToByte(actual_y);
                TextBox_debugOutput.Text = "ABS trigger value: " + dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.Simulate_ABS_value+"%";
                text_SABS.Text = "ABS trigger value: " + dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.Simulate_ABS_value + "%";
                Canvas.SetTop(text_SABS, y - rect_SABS_Control.Height-text_SABS.Height);
                Canvas.SetTop(rectangle, y-rect_SABS_Control.Height/2);

            }
        }
        private void Rectangle_MouseMove_sigle_slider_H(object sender, MouseEventArgs e)
        {
            if (isDragging)
            {
                var rectangle = sender as Rectangle;


                //damping
                if (rectangle.Name == "rect_damping")
                {
                    // Ensure the rectangle stays within the canvas
                    double damping_max = 255;
                    double x = e.GetPosition(canvas_horz_damping).X - offset.X;
                    double dx = canvas_horz_damping.Width / damping_max;
                    double min_position = 0 * dx;
                    double max_position = damping_max * dx;
                    //double dx = 100 / (canvas_horz_slider.Width - 10);
                    x = Math.Max(min_position, Math.Min(x, max_position));
                    double actual_x = x / dx;
                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.dampingPress = Convert.ToByte(actual_x);
                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.dampingPull = Convert.ToByte(actual_x);
                    text_damping.Text = "" + dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.dampingPress;
                    Canvas.SetLeft(text_damping, Canvas.GetLeft(rect_damping) + rect_damping.Width / 2-text_damping.Width/2);
                    Canvas.SetTop(text_damping, 5);
                    Canvas.SetLeft(rectangle, x);
                }
                // ABS Amplitude
                if (rectangle.Name == "rect_ABS")
                {
                    // Ensure the rectangle stays within the canvas
                    double x = e.GetPosition(canvas_horz_ABS).X - offset.X;
                    double ABS_max = 255;
                    double dx = canvas_horz_ABS.Width / ABS_max;
                    double min_position = 0 * dx;
                    double max_position = ABS_max * dx;
                    //double dx = 100 / (canvas_horz_slider.Width - 10);
                    x = Math.Max(min_position, Math.Min(x, max_position));
                    double actual_x = x / dx;
                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.absAmplitude = Convert.ToByte(actual_x);


                    switch (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.absForceOrTarvelBit)
                    {
                        case 0:
                            text_ABS.Text = (float)dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.absAmplitude / 20 + "kg";
                            break;
                        case 1:
                            text_ABS.Text = (float)dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.absAmplitude / 20 + "%";
                            break;
                        default:
                            break;
                    }

                    
                    Canvas.SetLeft(text_ABS, Canvas.GetLeft(rect_ABS) - text_ABS.Width / 2 + rect_ABS.Width / 2);
                    Canvas.SetTop(text_ABS, 5);
                    Canvas.SetLeft(rectangle, x);
                }
                //ABS freq
                if (rectangle.Name == "rect_ABS_freq")
                {
                    // Ensure the rectangle stays within the canvas
                    double x = e.GetPosition(canvas_horz_ABS_freq).X - offset.X;
                    double ABS_freq_max = 30;
                    double dx = canvas_horz_ABS_freq.Width / ABS_freq_max;
                    double min_position = 0 * dx;
                    double max_position = ABS_freq_max * dx;
                    //double dx = 100 / (canvas_horz_slider.Width - 10);
                    x = Math.Max(min_position, Math.Min(x, max_position));
                    double actual_x = x / dx;
                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.absFrequency = Convert.ToByte(actual_x);
                    text_ABS_freq.Text = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.absFrequency + "Hz";
                    Canvas.SetLeft(text_ABS_freq, Canvas.GetLeft(rect_ABS_freq) + rect_ABS_freq.Width / 2 - text_ABS_freq.Width / 2);
                    Canvas.SetTop(text_ABS_freq, 5);
                    Canvas.SetLeft(rectangle, x);
                }
                //max game output
                if (rectangle.Name == "rect_max_game")
                {
                    // Ensure the rectangle stays within the canvas
                    double x = e.GetPosition(canvas_horz_max_game).X - offset.X;
                    double max_game_max = 100;
                    double dx = canvas_horz_max_game.Width / max_game_max;
                    double min_position = 0 * dx;
                    double max_position = max_game_max * dx;
                    //double dx = 100 / (canvas_horz_slider.Width - 10);
                    x = Math.Max(min_position, Math.Min(x, max_position));
                    double actual_x = x / dx;
                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.maxGameOutput = Convert.ToByte(actual_x);

                    text_max_game.Text = "" + dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.maxGameOutput + "%";
                    Canvas.SetLeft(text_max_game, Canvas.GetLeft(rect_max_game) - text_max_game.Width / 2 + rect_max_game.Width / 2);
                    Canvas.SetTop(text_max_game, 5);
                    Canvas.SetLeft(rectangle, x);
                }
                //KF Slider

                if (rectangle.Name == "rect_KF")
                {
                    // Ensure the rectangle stays within the canvas
                    double x = e.GetPosition(canvas_horz_KF).X - offset.X;
                    double KF_max = 255;
                    double dx = canvas_horz_KF.Width / KF_max;
                    double min_position = 0 * dx;
                    double max_position = KF_max * dx;

                    x = Math.Max(min_position, Math.Min(x, max_position));
                    double actual_x = x / dx;
                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.kf_modelNoise = Convert.ToByte(actual_x);

                    text_KF.Text = "" + dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.kf_modelNoise;
                    Canvas.SetLeft(text_KF, Canvas.GetLeft(rect_KF) + rect_KF.Width / 2 - text_KF.Width / 2);
                    Canvas.SetTop(text_KF, 5);
                    Canvas.SetLeft(rectangle, x);
                }
                //LC rating slider
                if (rectangle.Name == "rect_LC_rating")
                {
                    // Ensure the rectangle stays within the canvas
                    double x = e.GetPosition(canvas_horz_LC_rating).X - offset.X;
                    double LC_max = 510;
                    double dx = canvas_horz_LC_rating.Width / LC_max;
                    double min_position = 0 * dx;
                    double max_position = LC_max * dx;

                    x = Math.Max(min_position, Math.Min(x, max_position));
                    double actual_x = x / dx;
                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.loadcell_rating = (byte)(actual_x / 2);

                    text_LC_rating.Text = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.loadcell_rating*2 + "kg";
                    Canvas.SetLeft(text_LC_rating, Canvas.GetLeft(rect_LC_rating) + rect_LC_rating.Width / 2 - text_LC_rating.Width / 2);
                    Canvas.SetTop(text_LC_rating, 5);
                    Canvas.SetLeft(rectangle, x);
                }
                // RPM effect AMP
                if (rectangle.Name == "rect_RPM_AMP")
                {
                    // Ensure the rectangle stays within the canvas
                    double x = e.GetPosition(canvas_horz_RPM_AMP).X - offset.X;
                    double RPM_AMP_max = 200;
                    double dx = (canvas_horz_RPM_AMP.Width -10)/ RPM_AMP_max;
                    double min_position = 0 * dx;
                    double max_position = RPM_AMP_max * dx;

                    x = Math.Max(min_position, Math.Min(x, max_position));
                    double actual_x = x / dx;
                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.RPM_AMP = (byte)(actual_x);

                    text_RPM_AMP.Text = ((float)dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.RPM_AMP) /100 + "kg";
                    Canvas.SetLeft(text_RPM_AMP, x - text_RPM_AMP.Width / 2 + rect_RPM_AMP.Width / 2);
                    Canvas.SetTop(text_RPM_AMP, 5);
                    Canvas.SetLeft(rectangle, x);
                }

                //Bite point control
                if (rectangle.Name == "rect_BP_Control")
                {
                    // Ensure the rectangle stays within the canvas
                    double x = e.GetPosition(canvas).X - offset.X;
                    double BP_max = 100;
                    double dx = (canvas.Width) / BP_max;
                    double min_position = 10 * dx - rect_BP_Control.Width / 2;
                    double max_position = (BP_max - 10) * dx - rect_BP_Control.Width / 2;

                    x = Math.Max(min_position, Math.Min(x, max_position));
                    double actual_x = (x + rect_BP_Control.Width / 2) / dx;
                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.BP_trigger_value = (byte)(actual_x);

                    text_BP.Text = "Bite Point:\n" + ((float)dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.BP_trigger_value) + "%";
                    Canvas.SetLeft(rectangle, x);
                    Canvas.SetLeft(text_BP, Canvas.GetLeft(rect_BP_Control) + rect_BP_Control.Width + 3);
                    Canvas.SetTop(text_BP, canvas.Height - text_BP.Height);

                }

                if (rectangle.Name == "rect_bite_amp")
                {
                    // Ensure the rectangle stays within the canvas
                    double x = e.GetPosition(canvas_horz_bite_amp).X - offset.X;
                    double bite_amp_max = 200;
                    double dx = canvas_horz_bite_amp.Width / bite_amp_max;
                    double min_position = 0 * dx;
                    double max_position = bite_amp_max * dx;
                    //double dx = 100 / (canvas_horz_slider.Width - 10);
                    x = Math.Max(min_position, Math.Min(x, max_position));
                    double actual_x = x / dx;
                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.BP_amp = Convert.ToByte(actual_x);
                    text_bite_amp.Text = ((float)dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.BP_amp) / 100.0f + "kg";
                    Canvas.SetLeft(text_bite_amp, Canvas.GetLeft(rect_bite_amp) + rect_bite_amp.Width / 2 - text_bite_amp.Width / 2);
                    Canvas.SetTop(text_bite_amp, 5);
                    Canvas.SetLeft(rectangle, x);
                }
                if (rectangle.Name == "rect_bite_freq")
                {
                    // Ensure the rectangle stays within the canvas
                    double x = e.GetPosition(canvas_horz_bite_freq).X - offset.X;
                    double bite_freq_max = 30;
                    double dx = canvas_horz_bite_freq.Width / bite_freq_max;
                    double min_position = 0 * dx;
                    double max_position = bite_freq_max * dx;
                    //double dx = 100 / (canvas_horz_slider.Width - 10);
                    x = Math.Max(min_position, Math.Min(x, max_position));
                    double actual_x = x / dx;
                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.BP_freq = Convert.ToByte(actual_x);
                    text_bite_freq.Text = (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.BP_freq) + "Hz";
                    Canvas.SetLeft(text_bite_freq, Canvas.GetLeft(rect_bite_freq) + rect_bite_freq.Width / 2 - text_bite_freq.Width / 2);
                    Canvas.SetTop(text_bite_freq, 5);
                    Canvas.SetLeft(rectangle, x);
                }
                //Pgain
                if (rectangle.Name == "rect_Pgain")
                {
                    // Ensure the rectangle stays within the canvas
                    double value_max = 2;
                    double x = e.GetPosition(canvas_horz_Pgain).X - offset.X;
                    double dx = canvas_horz_Pgain.Width / value_max;
                    double min_position = 0 * dx;
                    double max_position = value_max * dx;
                    //double dx = 100 / (canvas_horz_slider.Width - 10);
                    x = Math.Max(min_position, Math.Min(x, max_position));
                    double actual_x = x / dx;
                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.PID_p_gain = (float)actual_x;
                    text_Pgain.Text = "" + Math.Round(dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.PID_p_gain, 2);
                    Canvas.SetLeft(text_Pgain, Canvas.GetLeft(rect_Pgain) + rect_Pgain.Width / 2 - text_Pgain.Width / 2);
                    Canvas.SetTop(text_Pgain, 5);
                    Canvas.SetLeft(rectangle, x);
                }

                if (rectangle.Name == "rect_Igain")
                {
                    // Ensure the rectangle stays within the canvas
                    double value_max = 500;
                    double x = e.GetPosition(canvas_horz_Igain).X - offset.X;
                    double dx = canvas_horz_Igain.Width / value_max;
                    double min_position = 0 * dx;
                    double max_position = value_max * dx;
                    //double dx = 100 / (canvas_horz_slider.Width - 10);
                    x = Math.Max(min_position, Math.Min(x, max_position));
                    double actual_x = x / dx;
                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.PID_i_gain = (float)actual_x;
                    text_Igain.Text = "" + Math.Round(dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.PID_i_gain, 1);
                    Canvas.SetLeft(text_Igain, Canvas.GetLeft(rect_Igain) + rect_Igain.Width / 2 - text_Igain.Width / 2);
                    Canvas.SetTop(text_Igain, 5);
                    Canvas.SetLeft(rectangle, x);
                }

                if (rectangle.Name == "rect_Dgain")
                {
                    // Ensure the rectangle stays within the canvas
                    double value_max = 0.01;
                    double x = e.GetPosition(canvas_horz_Dgain).X - offset.X;
                    double dx = canvas_horz_Dgain.Width / value_max;
                    double min_position = 0 * dx;
                    double max_position = value_max * dx;
                    //double dx = 100 / (canvas_horz_slider.Width - 10);
                    x = Math.Max(min_position, Math.Min(x, max_position));
                    double actual_x = x / dx;
                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.PID_d_gain = (float)actual_x;
                    text_Dgain.Text = "" + Math.Round(dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.PID_d_gain, 4);
                    Canvas.SetLeft(text_Dgain, Canvas.GetLeft(rect_Dgain) + rect_Dgain.Width / 2 - text_Dgain.Width / 2);
                    Canvas.SetTop(text_Dgain, 5);
                    Canvas.SetLeft(rectangle, x);
                }

                if (rectangle.Name == "rect_VFgain")
                {
                    // Ensure the rectangle stays within the canvas
                    double value_max = 20;
                    double x = e.GetPosition(canvas_horz_VFgain).X - offset.X;
                    double dx = canvas_horz_VFgain.Width / value_max;
                    double min_position = 0 * dx;
                    double max_position = value_max * dx;
                    //double dx = 100 / (canvas_horz_slider.Width - 10);
                    x = Math.Max(min_position, Math.Min(x, max_position));
                    double actual_x = x / dx;
                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.PID_velocity_feedforward_gain = (float)actual_x;
                    text_VFgain.Text = "" + Math.Round(dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.PID_velocity_feedforward_gain, 2);
                    Canvas.SetLeft(text_VFgain, Canvas.GetLeft(rect_VFgain) + rect_VFgain.Width / 2 - text_VFgain.Width / 2);
                    Canvas.SetTop(text_VFgain, 5);
                    Canvas.SetLeft(rectangle, x);
                }

                if (rectangle.Name == "rect_G_force_multi")
                {
                    // Ensure the rectangle stays within the canvas
                    double x = e.GetPosition(canvas_horz_G_force_multi).X - offset.X;
                    double G_force_max = 100;
                    double dx = canvas_horz_G_force_multi.Width / G_force_max;
                    double min_position = 0 * dx;
                    double max_position = G_force_max * dx;
                    //double dx = 100 / (canvas_horz_slider.Width - 10);
                    x = Math.Max(min_position, Math.Min(x, max_position));
                    double actual_x = x / dx;
                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.G_multi = Convert.ToByte(actual_x);
                    text_G_multi.Text = (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.G_multi) + "%";
                    Canvas.SetLeft(text_G_multi, Canvas.GetLeft(rect_G_force_multi) + rect_G_force_multi.Width / 2 - text_G_multi.Width / 2);
                    Canvas.SetTop(text_G_multi, 5);
                    Canvas.SetLeft(rectangle, x);
                }
                //G_force window
                if (rectangle.Name == "rect_G_force_window")
                {
                    // Ensure the rectangle stays within the canvas
                    double x = e.GetPosition(canvas_horz_G_force_window).X - offset.X;
                    double G_window_max = 100;
                    double dx = canvas_horz_G_force_multi.Width / G_window_max;
                    double min_position = 10 * dx;
                    double max_position = G_window_max * dx;
                    //double dx = 100 / (canvas_horz_slider.Width - 10);
                    x = Math.Max(min_position, Math.Min(x, max_position));
                    double actual_x = x / dx;
                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.G_window = Convert.ToByte(actual_x);
                    text_G_window.Text = (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.G_window) + "";
                    Canvas.SetLeft(text_G_window, Canvas.GetLeft(rect_G_force_window) + rect_G_force_window.Width / 2 - text_G_window.Width / 2);
                    Canvas.SetTop(text_G_window, 5);
                    Canvas.SetLeft(rectangle, x);
                }


                //MPC 0th order gain
                if (rectangle.Name == "rect_MPC_0th_order_gain")
                {
                    // Ensure the rectangle stays within the canvas
                    double value_max = 4;
                    double x = e.GetPosition(canvas_horz_MPC_0th_order_gain).X - offset.X;
                    double dx = canvas_horz_MPC_0th_order_gain.Width / value_max;
                    double min_position = 0 * dx;
                    double max_position = value_max * dx;
                    //double dx = 100 / (canvas_horz_slider.Width - 10);
                    x = Math.Max(min_position, Math.Min(x, max_position));
                    double actual_x = x / dx;
                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.MPC_0th_order_gain = (float)actual_x;
                    text_MPC_0th_order_gain.Text = "" + Math.Round(dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.MPC_0th_order_gain, 2) + "mm/kg";
                    Canvas.SetLeft(text_MPC_0th_order_gain, Canvas.GetLeft(rect_MPC_0th_order_gain) + rect_MPC_0th_order_gain.Width / 2 - text_MPC_0th_order_gain.Width / 2);
                    Canvas.SetTop(text_MPC_0th_order_gain, 5);
                    Canvas.SetLeft(rectangle, x);
                }



                ////MPC 1st order gain
                //if (rectangle.Name == "rect_MPC_1st_order_gain")
                //{
                //    // Ensure the rectangle stays within the canvas
                //    double value_max = 0.01;
                //    double x = e.GetPosition(canvas_horz_MPC_1st_order_gain).X - offset.X;
                //    double dx = canvas_horz_MPC_1st_order_gain.Width / (2*value_max);
                //    double min_position = 0 * dx;
                //    double max_position = value_max * 2 * dx;
                //    //double dx = 100 / (canvas_horz_slider.Width - 10);
                //    x = Math.Max(min_position, Math.Min(x, max_position));
                //    double actual_x = x / dx;
                //    actual_x -= value_max;
                //    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.MPC_1st_order_gain = (float)actual_x;
                //    text_MPC_1st_order_gain.Text = "" + Math.Round(dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.MPC_1st_order_gain, 4);
                //    Canvas.SetLeft(text_MPC_1st_order_gain, Canvas.GetLeft(rect_MPC_1st_order_gain) + rect_MPC_1st_order_gain.Width / 2 - text_MPC_1st_order_gain.Width / 2);
                //    Canvas.SetTop(text_MPC_1st_order_gain, 5);
                //    Canvas.SetLeft(rectangle, x);
                //}

            }
        }
        private void Rectangle_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            if (isDragging)
            {
                var rectangle = sender as Rectangle;
                isDragging = false;
                rectangle.ReleaseMouseCapture();
                text_point_pos.Visibility = Visibility.Hidden;
                //SolidColorBrush buttonBackground = btn_update.Background as SolidColorBrush;
                //Color color = Color.FromArgb(150, buttonBackground.Color.R, buttonBackground.Color.G, buttonBackground.Color.B);
                //rectangle.Fill = btn_update.Background;
                if (rectangle.Name != "rect_SABS_Control" & rectangle.Name != "rect_BP_Control")
                {
                    var dropShadowEffect = new DropShadowEffect
                    {
                        ShadowDepth = 0,
                        BlurRadius = 20,
                        Color = Colors.White,
                        Opacity = 0
                    };
                    rectangle.Fill = defaultcolor;
                    rectangle.Effect = dropShadowEffect;
                }

                //rectangle.Fill = new SolidColorBrush(color);
            }
        }
        private void Debug_checkbox_Checked(object sender, RoutedEventArgs e)
        {

            text_debug_flag.Visibility = Visibility.Visible; 
            text_serial.Visibility = Visibility.Visible;
            TextBox_serialMonitor.Visibility = System.Windows.Visibility.Visible;
            textBox_debug_Flag_0.Visibility = Visibility.Visible;
            //btn_serial.Visibility = System.Windows.Visibility.Visible;
            btn_system_id.Visibility = System.Windows.Visibility.Visible;
            button_pedal_position_reset.Visibility = System.Windows.Visibility.Visible;
            button_pedal_restart.Visibility = System.Windows.Visibility.Visible;
            btn_reset_default.Visibility = System.Windows.Visibility.Visible;
            dump_pedal_response_to_file.Visibility = System.Windows.Visibility.Visible;
            InvertLoadcellReading_check.Visibility = Visibility.Visible;
            InvertMotorDir_check.Visibility = Visibility.Visible;
            //text_state.Visibility = Visibility.Hidden;
            debug_flag = true;
            Border_serial_monitor.Visibility = Visibility.Visible;
            debug_border.Visibility = Visibility.Visible;
            debug_label_text.Visibility = Visibility.Visible;

        }
        private void Debug_checkbox_Unchecked(object sender, RoutedEventArgs e)
        {
            text_debug_flag.Visibility = Visibility.Hidden; ;
            text_serial.Visibility = Visibility.Hidden;
            TextBox_serialMonitor.Visibility = System.Windows.Visibility.Hidden;
            textBox_debug_Flag_0.Visibility = Visibility.Hidden;
            //btn_serial.Visibility = System.Windows.Visibility.Hidden;
            btn_system_id.Visibility = System.Windows.Visibility.Hidden;
            button_pedal_position_reset.Visibility = System.Windows.Visibility.Hidden;
            button_pedal_restart.Visibility = System.Windows.Visibility.Hidden;
            btn_reset_default.Visibility = System.Windows.Visibility.Hidden;
            dump_pedal_response_to_file.Visibility = System.Windows.Visibility.Hidden;
            InvertLoadcellReading_check.Visibility = Visibility.Hidden;
            InvertMotorDir_check.Visibility = Visibility.Hidden;
            //text_state.Visibility = Visibility.Visible;
            debug_flag = false;
            Border_serial_monitor.Visibility = Visibility.Hidden;
            debug_border.Visibility = Visibility.Hidden;
            debug_label_text.Visibility = Visibility.Hidden;
        }


        private void JoystickOutput_checked(object sender, RoutedEventArgs e)
        {
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.travelAsJoystickOutput_u8 = 1;

        }
        private void JoystickOutput_unchecked(object sender, RoutedEventArgs e)
        {
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.travelAsJoystickOutput_u8 = 0;
        }


        private void InvertLoadcellReading_checked(object sender, RoutedEventArgs e)
        {
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.invertLoadcellReading_u8 = 1;
        }
        private void InvertLoadcellReading_unchecked(object sender, RoutedEventArgs e)
        {
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.invertLoadcellReading_u8 = 0;
        }


        private void InvertMotorDir_checked(object sender, RoutedEventArgs e)
        {
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.invertMotorDirection_u8 = 1;
        }
        private void InvertMotorDir_unchecked(object sender, RoutedEventArgs e)
        {
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.invertMotorDirection_u8 = 0;
        }



        private void CheckBox_Checked(object sender, RoutedEventArgs e)
        {
            Plugin.Settings.reading_config = 1;
        }
        private void CheckBox_Unchecked(object sender, RoutedEventArgs e)
        {
            Plugin.Settings.reading_config = 0;
        }

        private void checkbox_auto_connect_Checked(object sender, RoutedEventArgs e)
        {
            Plugin.Settings.auto_connect_flag = 1;
        }

        private void checkbox_auto_connect_Unchecked(object sender, RoutedEventArgs e)
        {
            Plugin.Settings.auto_connect_flag = 0 ;
        }

        private void checkbox_enable_ABS_Checked(object sender, RoutedEventArgs e)
        {
            Plugin.Settings.ABS_enable_flag[indexOfSelectedPedal_u] = 1;
            checkbox_enable_ABS.Content = "ABS/TC Effect Enabled";
        }
        private void checkbox_enable_ABS_Unchecked(object sender, RoutedEventArgs e)
        {
            Plugin.Settings.ABS_enable_flag[indexOfSelectedPedal_u] = 0;
            checkbox_enable_ABS.Content = "ABS/TC Effect Disabled";
        }

        private void checkbox_enable_RPM_Checked(object sender, RoutedEventArgs e)
        {
            Plugin.Settings.RPM_enable_flag[indexOfSelectedPedal_u] = 1;
            checkbox_enable_RPM.Content = "Effect Enabled";
        }

        private void checkbox_enable_RPM_Unchecked(object sender, RoutedEventArgs e)
        {
            Plugin.Settings.RPM_enable_flag[indexOfSelectedPedal_u] = 0;
            checkbox_enable_RPM.Content = "Effect Disabled";
        }

        private void Vjoy_out_check_Checked(object sender, RoutedEventArgs e)
        {
            Plugin.Settings.vjoy_output_flag = 1;
            ////// vJoy c# wrapper, see https://github.com/bobhelander/vJoy.Wrapper
            ////uint vJoystickId = Plugin.Settings.vjoy_order;
            ////joystick = new VirtualJoystick(Plugin.Settings.vjoy_order);
            ////joystick.Aquire();
            ////vjoy_axis_initialize();

            uint vJoystickId = Plugin.Settings.vjoy_order;
            //joystick = new VirtualJoystick(Plugin.Settings.vjoy_order);
            joystick = new vJoyInterfaceWrap.vJoy();

            joystick.AcquireVJD(vJoystickId);
            //joystick.Aquire();
            vjoy_axis_initialize();

        }


        private void Vjoy_out_check_Unchecked(object sender, RoutedEventArgs e)
        {
            Plugin.Settings.vjoy_output_flag = 0;
            //joystick.Release();
            joystick.RelinquishVJD(Plugin.Settings.vjoy_order);
        }


        public void EffectAppliedOnForceOrTravel_combobox_changed(object sender, SelectionChangedEventArgs e)
        {
            try
            {
                dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.absForceOrTarvelBit = (byte)EffectAppliedOnForceOrTravel_combobox.SelectedIndex;

                if (text_ABS != null)
                {
                    switch (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.absForceOrTarvelBit)
                    {
                        case 0:
                            text_ABS.Text = (float)dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.absAmplitude / 20 + "kg";
                            break;
                        case 1:
                            text_ABS.Text = (float)dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.absAmplitude / 20 + "%";
                            break;
                        default:
                            break;
                    }
                }
                

            }
            catch (Exception caughtEx)
            {
                string errorMessage = caughtEx.Message;
                TextBox_debugOutput.Text = errorMessage;
            }

        }

        private void vjoy_plus_click(object sender, RoutedEventArgs e)
        {
            // release old joystick
            joystick.RelinquishVJD(Plugin.Settings.vjoy_order);

            Plugin.Settings.vjoy_order += 1;
            uint max = 16;
            uint min = 1;
            Plugin.Settings.vjoy_order = Math.Max(min, Math.Min(Plugin.Settings.vjoy_order, max));
            Label_vjoy_order.Content = Plugin.Settings.vjoy_order;
            if (Plugin.Settings.vjoy_output_flag == 1)
            {
                //joystick.Release();
                
                //VjdStat status;
                VjdStat status = joystick.GetVJDStatus(Plugin.Settings.vjoy_order);
                //status = joystick.Joystick.GetVJDStatus(Plugin.Settings.vjoy_order);
                switch (status)
                {
                    case VjdStat.VJD_STAT_OWN:
                        TextBox_debugOutput.Text = "vjoy already aquaried";
                        Plugin.Settings.vjoy_output_flag = 0;
                        Vjoy_out_check.IsChecked = false;
                        break;
                    case VjdStat.VJD_STAT_FREE:

                        TextBox_debugOutput.Text = "vjoy aquaried";
                        //joystick = new VirtualJoystick(Plugin.Settings.vjoy_order);
                        //joystick.Aquire();
                        joystick.AcquireVJD(Plugin.Settings.vjoy_order);
                        if (Vjoy_out_check.IsChecked == false)
                        {
                            Vjoy_out_check.IsChecked = true;
                        }
                        //Console.WriteLine("vJoy Device {0} is free\n", id);
                        break;
                    case VjdStat.VJD_STAT_BUSY:
                        TextBox_debugOutput.Text = "vjoy was aquaried by other program";
                        Plugin.Settings.vjoy_output_flag = 0;
                        Vjoy_out_check.IsChecked = false;
                        //Console.WriteLine("vJoy Device {0} is already owned by another feeder\nCannot continue\n", id);
                        return;
                    case VjdStat.VJD_STAT_MISS:
                        TextBox_debugOutput.Text = "the selected vjoy device not enabled";
                        Plugin.Settings.vjoy_output_flag = 0;
                        Vjoy_out_check.IsChecked = false;
                        //Console.WriteLine("vJoy Device {0} is not installed or disabled\nCannot continue\n", id);
                        return;
                    default:
                        TextBox_debugOutput.Text = "vjoy device error";
                        Plugin.Settings.vjoy_output_flag = 0;
                        Vjoy_out_check.IsChecked = false;
                        //Console.WriteLine("vJoy Device {0} general error\nCannot continue\n", id);
                        return;
                };
            }
            

        }

        private void vjoy_minus_click(object sender, RoutedEventArgs e)
        {
            Plugin.Settings.vjoy_order -= 1;
            uint max = 16;
            uint min = 1;
            Plugin.Settings.vjoy_order = Math.Max(min, Math.Min(Plugin.Settings.vjoy_order, max));
            Label_vjoy_order.Content = Plugin.Settings.vjoy_order;
            if (Plugin.Settings.vjoy_output_flag == 1)
            {
                //joystick.Release();
                joystick.RelinquishVJD(Plugin.Settings.vjoy_order);
                VjdStat status;
                status = joystick.GetVJDStatus(Plugin.Settings.vjoy_order);
                switch (status)
                {
                    case VjdStat.VJD_STAT_OWN:
                        TextBox_debugOutput.Text = "vjoy already aquaried";
                        Plugin.Settings.vjoy_output_flag = 0;
                        Vjoy_out_check.IsChecked = false;
                        break;
                    case VjdStat.VJD_STAT_FREE:

                        TextBox_debugOutput.Text = "vjoy aquaried";
                        //joystick = new VirtualJoystick(Plugin.Settings.vjoy_order);
                        joystick.AcquireVJD(Plugin.Settings.vjoy_order);
                        //joystick.Aquire();
                        if (Vjoy_out_check.IsChecked == false)
                        {
                            Vjoy_out_check.IsChecked = true;
                        }
                        //Console.WriteLine("vJoy Device {0} is free\n", id);
                        break;
                    case VjdStat.VJD_STAT_BUSY:
                        TextBox_debugOutput.Text = "vjoy was aquaried by other program";
                        Plugin.Settings.vjoy_output_flag = 0;
                        Vjoy_out_check.IsChecked = false;
                        //Console.WriteLine("vJoy Device {0} is already owned by another feeder\nCannot continue\n", id);
                        return;
                    case VjdStat.VJD_STAT_MISS:
                        TextBox_debugOutput.Text = "the selected vjoy device not enabled";
                        Plugin.Settings.vjoy_output_flag = 0;
                        Vjoy_out_check.IsChecked = false;
                        //Console.WriteLine("vJoy Device {0} is not installed or disabled\nCannot continue\n", id);
                        return;
                    default:
                        TextBox_debugOutput.Text = "vjoy device error";
                        Plugin.Settings.vjoy_output_flag = 0;
                        Vjoy_out_check.IsChecked = false;
                        //Console.WriteLine("vJoy Device {0} general error\nCannot continue\n", id);
                        return;
                };
            }




        }
        private void checkbox_enable_bite_point_Checked(object sender, RoutedEventArgs e)
        {

            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.BP_trigger = 1;
            text_BP.Visibility = Visibility.Visible;
            rect_BP_Control.Visibility = Visibility.Visible;
            checkbox_enable_bite_point.Content = "Bite Point Vibration Enabled";


        }

        private void checkbox_enable_bite_point_Unchecked(object sender, RoutedEventArgs e)
        {

            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.BP_trigger = 0;
            text_BP.Visibility = Visibility.Hidden;
            rect_BP_Control.Visibility = Visibility.Hidden;
            checkbox_enable_bite_point.Content = "Bite Point Vibration Disabled";


        }

        private void btn_reset_default_Click(object sender, RoutedEventArgs e)
        {
            DAP_config_set_default(indexOfSelectedPedal_u);
            updateTheGuiFromConfig();
        }

        private void checkbox_enable_G_force_Unchecked(object sender, RoutedEventArgs e)
        {
            Plugin.Settings.G_force_enable_flag[indexOfSelectedPedal_u] = 0;
            checkbox_enable_G_force.Content = "G Force Effect Disabled";


        }
        private void checkbox_enable_G_force_Checked(object sender, RoutedEventArgs e)
        {
            Plugin.Settings.G_force_enable_flag[indexOfSelectedPedal_u] = 1;
            checkbox_enable_G_force.Content = "G Force Effect Enabled";


        }

        // for ocntrol strategy
        private void StrategySel(object sender, RoutedEventArgs e)
        {
            if (ControlStrategy_Sel_1.IsChecked == true)
            {
                dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.control_strategy_b = 0;
            }

            if (ControlStrategy_Sel_2.IsChecked == true)
            {
                dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.control_strategy_b = 1;
            }

            if (ControlStrategy_Sel_3.IsChecked == true)
            {
                dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.control_strategy_b = 2;
            }

        }
        // RPM effect select
        private void RPMeffecttype_Sel_1_Checked(object sender, RoutedEventArgs e)
        {
            if (RPMeffecttype_Sel_1.IsChecked == true)
            {
                Plugin.Settings.RPM_effect_type = 0;
                //TextBox_debugOutput.Text = "" + Plugin.Settings.RPM_effect_type;
            }
            if (RPMeffecttype_Sel_2.IsChecked == true)
            {
                Plugin.Settings.RPM_effect_type = 1;
                //TextBox_debugOutput.Text = "" + Plugin.Settings.RPM_effect_type;
            }
        }

        private void TabControl_file_path(object sender, SelectionChangedEventArgs e)
        {
            profile_select = (uint)ProfileTab.SelectedIndex;
            Plugin.profile_index = profile_select;
            //Plugin.Settings.table_selected = (uint)MyTab.SelectedIndex;
            // update the sliders & serial port selection accordingly
            updateTheGuiFromConfig();
        }

        private void file_check_Checked(object sender, RoutedEventArgs e)
        {
            var checkbox = sender as System.Windows.Controls.CheckBox;
            if (checkbox.Name == "Clutch_file_check")
            {
                Plugin.Settings.file_enable[profile_select, 0] = 1;
            }

            if (checkbox.Name == "Brake_file_check")
            {
                Plugin.Settings.file_enable[profile_select, 1] = 1;
            }

            if (checkbox.Name == "Gas_file_check")
            {
                Plugin.Settings.file_enable[profile_select, 2] = 1;
            }
        }

        private void file_check_Unchecked(object sender, RoutedEventArgs e)
        {
            var checkbox = sender as System.Windows.Controls.CheckBox;
            if (checkbox.Name == "Clutch_file_check")
            {
                Plugin.Settings.file_enable[profile_select, 0] = 0;
            }

            if (checkbox.Name == "Brake_file_check")
            {
                Plugin.Settings.file_enable[profile_select, 1] = 0;
            }

            if (checkbox.Name == "Gas_file_check")
            {
                Plugin.Settings.file_enable[profile_select, 2] = 0;
            }
        }

        public void Profile_change(uint profile_index)
        {
            profile_select = profile_index;
            ProfileTab.SelectedIndex = (int)profile_index;
            //if (Plugin.Settings.file_enable[profile_select])
            Parsefile(profile_index);
            string tmp;
            switch (profile_index)
            {
                case 0:
                    tmp = "Profile A";
                    break;
                case 1:
                    tmp = "Profile B";
                    break;
                case 2:
                    tmp = "Profile C";
                    break;
                case 3:
                    tmp = "Profile D";
                    break;
                default:
                    tmp = "No Profile";
                    break;
            }
            Plugin.current_profile = tmp;
        }
       



        /*
private void GetRectanglePositions()
{
   foreach (var kvp in rectanglePositions)
   {
       Console.WriteLine($"{kvp.Key}: X={kvp.Value.X}, Y={kvp.Value.Y}");
   }
}
*/

    }
    
}
