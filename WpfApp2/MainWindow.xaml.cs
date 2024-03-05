﻿using DeckLinkAPI;
using DirectShowLib;
using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace WpfApp2
{
    public partial class MainWindow : System.Windows.Window
    {
        private readonly EventWaitHandle m_applicationCloseWaitHandle;

        private Thread m_deckLinkMainThread;

        private DeckLinkDeviceDiscovery m_deckLinkDeviceDiscovery;
        private DeckLinkDevice m_inputDevice1;
        private DeckLinkDevice m_inputDevice2;

        private DeckLinkOutputDevice m_outputDevice;

        private ProfileCallback m_profileCallback;

        private CaptureCallback m_captureCallback1;
        private CaptureCallback m_captureCallback2;

        private PlaybackCallback m_playbackCallback;

        private int cameraIndex = -1;

        private VideoCapture cameraCapture;
        private bool captureRunning;
        private Thread captureThread;

        private Mat latestCameraFrame;

        private Mat latestDeckLinkFrame1;
        private Mat latestDeckLinkFrame2;

        private object frameLock = new object();

        public MainWindow()
        {
            InitializeComponent();
            m_applicationCloseWaitHandle = new EventWaitHandle(false, EventResetMode.AutoReset);
        }

        private void DeckLinkMainThread()
        {
            m_profileCallback = new ProfileCallback();
            m_captureCallback1 = new CaptureCallback();
            m_captureCallback2 = new CaptureCallback();
            m_playbackCallback = new PlaybackCallback();

            m_captureCallback1.FrameReceived += OnFrameReceived1;
            m_captureCallback2.FrameReceived += OnFrameReceived2;

            m_deckLinkDeviceDiscovery = new DeckLinkDeviceDiscovery();
            m_deckLinkDeviceDiscovery.DeviceArrived += AddDevice;
            m_deckLinkDeviceDiscovery.Enable();

            m_applicationCloseWaitHandle.WaitOne();

            m_captureCallback1.FrameReceived -= OnFrameReceived1;
            m_captureCallback2.FrameReceived -= OnFrameReceived2;

            DisposeDeckLinkResources();
        }

        private void FindCameraIndex()
        {
            DsDevice[] cameras = DsDevice.GetDevicesOfCat(FilterCategory.VideoInputDevice);
            for (int i = 0; i < cameras.Length; i++)
            {
                if (cameras[i].Name == "HT-GE202GC-T-CL")
                {
                    cameraIndex = i;
                    break;
                }
            }

            if (cameraIndex == -1)
                throw new Exception("Camera not found");
        }


        private void AddDevice(object sender, DeckLinkDiscoveryEventArgs e)
        {
            var deviceName = DeckLinkDeviceTools.GetDisplayLabel(e.deckLink);
            if (deviceName.Contains("DeckLink Duo (2)"))
            {
                m_inputDevice1 = new DeckLinkDevice(e.deckLink, m_profileCallback);
                InitializeInputDevice1();
            }
            else if (deviceName.Contains("DeckLink Duo (3)"))
            {
                m_inputDevice2 = new DeckLinkDevice(e.deckLink, m_profileCallback);
                InitializeInputDevice2();
            }
            else if (deviceName.Contains("DeckLink Duo (4)"))
            {
                m_outputDevice = new DeckLinkOutputDevice(e.deckLink, m_profileCallback);
                InitializeOutputDevice();
            }
        }


        private void InitializeInputDevice1()
        {
            if (m_inputDevice1 != null)
            {
                m_inputDevice1.StartCapture(_BMDDisplayMode.bmdModeHD1080p5994, m_captureCallback1, false);
            }
        }

        private void InitializeInputDevice2()
        {
            if (m_inputDevice2 != null)
            {
                m_inputDevice2.StartCapture(_BMDDisplayMode.bmdModeHD1080p5994, m_captureCallback2, false);
            }
        }

        private void InitializeOutputDevice()
        {
            if (m_outputDevice != null)
            {
                m_outputDevice.PrepareForPlayback(_BMDDisplayMode.bmdModeHD1080p6000, m_playbackCallback);
            }

        }

        private Mat ProcessFrameWithOpenCV(Mat inputFrame)
        {
            //double alpha = 1; 
            //double beta = 0;    
            //Mat contrastAdjustedFrame = AdjustContrastUYVY(inputFrame, alpha, beta);

            //Mat finalFrame = DrawRectangle(contrastAdjustedFrame);
            Mat finalFrame = DrawRectangle(inputFrame);

            return finalFrame;
        }

        private Mat DrawRectangle(Mat inputFrame)
        {
            int rectWidth = 400;
            int rectHeight = 250;
            int centerX = inputFrame.Width / 2;
            int centerY = inputFrame.Height / 2;

            int leftX = centerX - rectWidth / 2;
            int rightX = centerX + rectWidth / 2;
            int bottomY = centerY + rectHeight / 2;

            OpenCvSharp.Point leftTop = new OpenCvSharp.Point(leftX, centerY - rectHeight / 2);
            OpenCvSharp.Point leftBottom = new OpenCvSharp.Point(leftX, bottomY);
            OpenCvSharp.Point rightTop = new OpenCvSharp.Point(rightX, centerY - rectHeight / 2);
            OpenCvSharp.Point rightBottom = new OpenCvSharp.Point(rightX, bottomY);

            Scalar greenColor = new Scalar(0, 255, 0);
            int thickness = 2;

            Cv2.Line(inputFrame, leftTop, leftBottom, greenColor, thickness);
            Cv2.Line(inputFrame, rightTop, rightBottom, greenColor, thickness);

            Cv2.Line(inputFrame, leftBottom, rightBottom, greenColor, thickness);

            return inputFrame;
        }

        //private Mat AdjustContrastUYVY(Mat uyvyFrame, double alpha, double beta)
        //{
        //    // UYVY format: U0 Y0 V0 Y1 (for each two pixels)
        //    // Create a clone to work on
        //    Mat adjustedFrame = uyvyFrame.Clone();

        //    unsafe
        //    {
        //        byte* dataPtr = (byte*)adjustedFrame.DataPointer;
        //        int totalBytes = uyvyFrame.Rows * uyvyFrame.Cols * uyvyFrame.ElemSize();

        //        for (int i = 0; i < totalBytes; i += 4)
        //        {
        //            // Adjust Y values at positions 1 and 3 in the UYVY sequence
        //            for (int j = 1; j <= 3; j += 2)
        //            {
        //                int yValue = dataPtr[i + j];
        //                // Adjust the Y value
        //                yValue = (int)(alpha * yValue + beta);
        //                // Clamp the value to 0-255 range
        //                yValue = Math.Max(0, Math.Min(255, yValue));
        //                dataPtr[i + j] = (byte)yValue;
        //            }
        //        }
        //    }

        //    return adjustedFrame;
        //}


        private void StartCameraCapture()
        {
            if (cameraIndex == -1) return; 

            cameraCapture = new VideoCapture(cameraIndex)
            {
                FrameWidth = 1920,
                FrameHeight = 1080

            };
            cameraCapture.Set(VideoCaptureProperties.Fps, 60);
            captureRunning = true;
            captureThread = new Thread(CaptureCamera);
            captureThread.Start();
        }



        private Mat ProcessCameraFrame(Mat frame)
        {
            return ResizeFrame(frame, 0.25); //for 3
        }

        private Mat ConvertBGRToUYVY(Mat bgrFrame)
        {
            // Convert from BGR to YUV
            Mat yuvFrame = new Mat();
            Cv2.CvtColor(bgrFrame, yuvFrame, ColorConversionCodes.BGR2YUV);

            int rows = yuvFrame.Rows;
            int cols = yuvFrame.Cols;

            // Create UYVY (YUV 4:2:2) format image
            Mat uyvyFrame = new Mat(rows, cols, MatType.CV_8UC2);

            Parallel.For(0, rows, y =>
            {
                for (int x = 0; x < cols; x += 2)
                {
                    Vec3b pixel1 = yuvFrame.At<Vec3b>(y, x);
                    Vec3b pixel2 = yuvFrame.At<Vec3b>(y, x + 1);

                    Vec2b uyvyPixel1 = new Vec2b
                    {
                        Item0 = pixel1[1], // U
                        Item1 = pixel1[0]  // Y0
                    };
                    uyvyFrame.Set(y, x, uyvyPixel1);

                    Vec2b uyvyPixel2 = new Vec2b
                    {
                        Item0 = pixel1[2], // V (using U from pixel1)
                        Item1 = pixel2[0]  // Y1
                    };
                    uyvyFrame.Set(y, x + 1, uyvyPixel2);
                }
            });

            return uyvyFrame;
        }

        private Mat ConvertUYVYToBGR(Mat uyvyFrame)
        {
            Mat bgrFrame = new Mat();
            Cv2.CvtColor(uyvyFrame, bgrFrame, ColorConversionCodes.YUV2BGR_UYVY);
            return bgrFrame;
        }

        private void CaptureCamera()
        {
            while (captureRunning)
            {
                using (Mat camframe = new Mat())
                {
                    if (cameraCapture.Read(camframe)) 
                    {
                        Mat processedFrame = ProcessCameraFrame(camframe);
                        //m_outputDevice.ScheduleFrame(processedFrame);

                        lock (frameLock)
                        {
                            latestCameraFrame = processedFrame.Clone();
                        }

                        ProcessAndOutputCombinedFrame();
                    }
                }
            }
        }
        
        private Mat CropFrame(Mat originalFrame, int cropLeft, int cropRight)
        {
            int width = originalFrame.Width - cropLeft - cropRight;
            if (width > 0)
            {
                using (Mat croppedFrame = new Mat(originalFrame, new OpenCvSharp.Rect(cropLeft, 0, width, originalFrame.Height)))
                {
                    return croppedFrame.Clone();
                }
            }
            else
            {
                throw new ArgumentException("Crop parameters exceed the width of the original frame.");
            }
        }

        private Mat ResizeFrame(Mat originalFrame, double scale)
        {
            OpenCvSharp.Size newSize = new OpenCvSharp.Size((int)(originalFrame.Width * scale), (int)(originalFrame.Height * scale));
            using (Mat resizedFrame = new Mat())
            {
                Cv2.Resize(originalFrame, resizedFrame, newSize);
                return resizedFrame.Clone();
            }
        }

        private void OnFrameReceived1(IDeckLinkVideoInputFrame videoFrame)
        {
            IntPtr frameBytes;
            videoFrame.GetBytes(out frameBytes);
            Mat frame = new Mat(videoFrame.GetHeight(), videoFrame.GetWidth(), MatType.CV_8UC2, frameBytes);

            Mat bgrFrame = ConvertUYVYToBGR(frame);
            // Crop the frame to 1440x1080 by cutting 240 pixels from both left and right sides
            Mat croppedFrame = CropFrame(bgrFrame, 240, 240);

            lock (frameLock)
            {
                latestDeckLinkFrame1 = croppedFrame.Clone();
            }

            ProcessAndOutputCombinedFrame();
        }

        private void OnFrameReceived2(IDeckLinkVideoInputFrame videoFrame)
        {
            IntPtr frameBytes;
            videoFrame.GetBytes(out frameBytes);
            Mat frame = new Mat(videoFrame.GetHeight(), videoFrame.GetWidth(), MatType.CV_8UC2, frameBytes);

            Mat bgrFrame = ConvertUYVYToBGR(frame);

            Mat resizedFrame = ResizeFrame(bgrFrame, 0.25);

            lock (frameLock)
            {
                latestDeckLinkFrame2 = resizedFrame.Clone();
            }

            ProcessAndOutputCombinedFrame();
        }


        private void ProcessAndOutputCombinedFrame()
        {
            lock (frameLock)
            {
                if (latestCameraFrame != null && latestDeckLinkFrame1 != null && latestDeckLinkFrame2 != null)
                {
                    using (Mat finalFrame = new Mat(1080, 1920, MatType.CV_8UC3, new Scalar(0, 0, 0)))
                    {
                        var rectDeckLink1 = new OpenCvSharp.Rect(0, 0, latestDeckLinkFrame1.Width, latestDeckLinkFrame1.Height);
                        latestDeckLinkFrame1.CopyTo(finalFrame[rectDeckLink1]);

                        int startX = latestDeckLinkFrame1.Width;
                        var rectCamera = new OpenCvSharp.Rect(startX, 200, latestCameraFrame.Width, latestCameraFrame.Height);
                        latestCameraFrame.CopyTo(finalFrame[rectCamera]);

                        var rectDeckLink2 = new OpenCvSharp.Rect(startX, 640, latestDeckLinkFrame2.Width, latestDeckLinkFrame2.Height);
                        latestDeckLinkFrame2.CopyTo(finalFrame[rectDeckLink2]);

                        Mat uyvyFrame = ConvertBGRToUYVY(finalFrame);
                        m_outputDevice.ScheduleFrame(uyvyFrame);
                        uyvyFrame.Dispose();
                    }

                    latestCameraFrame.Dispose();
                    latestDeckLinkFrame1.Dispose();
                    latestDeckLinkFrame2.Dispose();
                    latestCameraFrame = null;
                    latestDeckLinkFrame1 = null;
                    latestDeckLinkFrame2 = null;
                }
            }
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            FindCameraIndex();
            StartCameraCapture();
            m_deckLinkMainThread = new Thread(() => DeckLinkMainThread());
            m_deckLinkMainThread.SetApartmentState(ApartmentState.MTA);
            m_deckLinkMainThread.Start();
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            StopCameraCapture();
            m_applicationCloseWaitHandle.Set();

            if (m_deckLinkMainThread != null && m_deckLinkMainThread.IsAlive)
            {
                m_deckLinkMainThread.Join();
            }

            DisposeDeckLinkResources();
        }

        private void StopCameraCapture()
        {
            captureRunning = false;
            captureThread?.Join();
            cameraCapture?.Dispose();
        }

        private void DisposeDeckLinkResources()
        {
            if (m_inputDevice1 != null)
            {
                m_inputDevice1.StopCapture();
                m_inputDevice1 = null;
            }

            if (m_inputDevice2 != null)
            {
                m_inputDevice2.StopCapture();
                m_inputDevice2 = null;
            }

            if (m_outputDevice != null)
            {
                m_outputDevice.StopPlayback();
                m_outputDevice = null;
            }

            if (m_deckLinkDeviceDiscovery != null)
            {
                m_deckLinkDeviceDiscovery.Disable();
                m_deckLinkDeviceDiscovery = null;
            }
        }
    }

    public class CaptureCallback : IDeckLinkInputCallback
    {
        public event Action<IDeckLinkVideoInputFrame> FrameReceived;

        public void VideoInputFrameArrived(IDeckLinkVideoInputFrame videoFrame, IDeckLinkAudioInputPacket audioPacket)
        {
            FrameReceived?.Invoke(videoFrame);
        }

        public void VideoInputFormatChanged(_BMDVideoInputFormatChangedEvents notificationEvents, IDeckLinkDisplayMode newDisplayMode, _BMDDetectedVideoInputFormatFlags detectedSignalFlags)
        {
        }
    }

    public class PlaybackCallback : IDeckLinkVideoOutputCallback
    {
        public event Action<IDeckLinkVideoFrame, _BMDOutputFrameCompletionResult> FrameCompleted;

        public void ScheduledFrameCompleted(IDeckLinkVideoFrame completedFrame, _BMDOutputFrameCompletionResult result)
        {
            FrameCompleted?.Invoke(completedFrame, result);
        }

        public void ScheduledPlaybackHasStopped()
        {
        }
    }
}
