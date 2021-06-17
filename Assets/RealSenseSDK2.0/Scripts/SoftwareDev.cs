using System;
using System.Threading;
using System.Collections.Generic;
using UnityEngine;
using Intel.RealSense;
using System.Collections;
using Intel.RealSense.Base;

public class SoftwareDev : RsFrameProvider
{
    public enum ProcessMode
    {
        Multithread,
        UnityThread,
    }
    public ProcessMode processMode;
    public override event Action<PipelineProfile> OnStart;
    public override event Action OnStop;
    public override event Action<Frame> OnNewSample;
    private Syncer sync;
    private Colorizer colorizer;

    private Intrinsics depthIntrinsics;
    private Intrinsics colorIntrinsics;

    public int frame_number;

    public RsConfiguration DeviceConfiguration = new RsConfiguration {
        mode = RsConfiguration.Mode.Live,
        RequestedSerialNumber = string.Empty
    };

    private Thread worker;
    private readonly AutoResetEvent stopEvent = new AutoResetEvent(false);
    private Pipeline m_pipeline;
    private static RosBridge rosConnector;
    private SoftwareDevice software_device;
    public SoftwareSensor depth_sensor;
    public SoftwareSensor color_sensor;
    public VideoStreamProfile depthProfile;
    public VideoStreamProfile colorProfile;
    public new PipelineProfile ActiveProfile;
    public SoftwareSensor[] Sensors;
    public VideoStreamProfile[] Streams;
    public Context context;
  
    PointCloud pb;
    private readonly object _lock = new object();

    void onEnable()
    {
        pb = new PointCloud();
        Debug.Log("starting onEnable()");
        context = new Context();
        rosConnector = GameObject.Find("RosConnector").gameObject.GetComponent<RosBridge>();
        m_pipeline = new Pipeline(context);
        // there needs to be a wait until this is properly populated inside RosBridge
        while(!rosConnector.depthIntrinsicsSet || !rosConnector.colorIntrinsicsSet) {

        }
        depthIntrinsics = rosConnector.getDepthCameraIntrinsics();
        colorIntrinsics = rosConnector.getColorCameraIntrinsics();
        
        var cfg = new Config();
        sync = new Syncer();
        cfg.EnableStream(Stream.Depth, depthIntrinsics.width, depthIntrinsics.height, Format.Z16, 30); //necessary?
        cfg.EnableStream(Stream.Color, colorIntrinsics.width, colorIntrinsics.height, Format.Rgb8, 30); //necessary?
        processMode = ProcessMode.Multithread;
    
        colorizer = new Colorizer();

        software_device = new SoftwareDevice();
        software_device.AddTo(context);
        ActiveProfile = m_pipeline.Start();
        
        depth_sensor = software_device.AddSensor("Depth");
        depthProfile = depth_sensor.AddVideoStream(new SoftwareVideoStream
        {
            type = Stream.Depth,
            index = 0,
            uid = 100,
            width = depthIntrinsics.width,
            height = depthIntrinsics.height,
            fps = 4,
            bpp = 2, //mono16
            format = Format.Z16, 
            intrinsics = rosConnector.getDepthCameraIntrinsics() //might have to dress this Intrinsics up as a diff type
        });
        // Debug.Log(depthProfile.Height + " software device depth height");
        // Debug.Log(depthProfile.Width + " software device depth width");
        depth_sensor.AddReadOnlyOption(Option.DepthUnits, 1.0f / 5000);
        
        color_sensor = software_device.AddSensor("Color");
        colorProfile = color_sensor.AddVideoStream(new SoftwareVideoStream
        {
            type = Stream.Color,
            index = 0,
            uid = 101,
            width = colorIntrinsics.width,
            height = colorIntrinsics.height,
            fps = 4,
            bpp = 3, 
            format = Format.Rgb8,
            intrinsics = rosConnector.getColorCameraIntrinsics()
        });
        // Debug.Log(colorProfile.Height + " software device color height");
        // Debug.Log(colorProfile.Width + " software device color width");
        frame_number = 0;
        software_device.SetMatcher(Matchers.Default);
        Sensors = new SoftwareSensor[2];
        Sensors[0] = depth_sensor;
        Sensors[1] = color_sensor;
        Streams = new VideoStreamProfile[2];
        Streams[0] = depthProfile;
        Streams[1] = colorProfile;
        //obj.Device.Sensors.FirstOrDefault().As<Sensor>().StreamProfiles.FirstOrDefault(s => s.Stream == Stream.Depth && s.Format == Format.Z16).As<VideoStreamProfile>()
        //
        //UnityEngine.Debug.Log(ActiveProfile.Device.Sensors[0].StreamProfiles[0].As<VideoStreamProfile>());
        // raw depth-> metric units translation for Colorizer class
        //this might be something we need from ROS is it in camera_info topic?
        //var rDepthSensor = ActiveProfile.Device.QuerySensors().First(s => s.Is(Extension.DepthSensor));
        depth_sensor.AddReadOnlyOption(Option.DepthUnits, 0.001f /*replace if theres a diff standard*/); //0.001f
                                                                                                         // whats the depth metric units
        depth_sensor.Open(depthProfile);
        color_sensor.Open(colorProfile);

        depth_sensor.Start(sync.SubmitFrame);
        color_sensor.Start(sync.SubmitFrame);
        //processMode = ProcessMode.Multithread;
        if (processMode == ProcessMode.Multithread)
        {
            stopEvent.Reset();
            worker = new Thread(WaitForFrames);
            worker.IsBackground = true;
            worker.Start();
        }
        StartCoroutine(WaitAndStart());
    }

    IEnumerator WaitAndStart()
    {
        yield return new WaitForEndOfFrame();
        Streaming = true;
        if (OnStart != null)
            OnStart(ActiveProfile);
    }

    void OnDisable()
    {
        OnNewSample = null;
        // OnNewSampleSet = null;

        if (worker != null)
        {
            stopEvent.Set();
            worker.Join();
        }

        if (Streaming && OnStop != null)
            OnStop();

        if (ActiveProfile != null)
        {
            ActiveProfile.Dispose();
            ActiveProfile = null;
        }
        
        if (m_pipeline != null)
        {
            if (Streaming)
            m_pipeline.Stop();
            m_pipeline.Dispose();
            m_pipeline = null;
        }

        Streaming = false;
    }

    void OnDestroy()
    {
        OnStart = null;
        OnStop = null;

        if (ActiveProfile != null)
        {
            ActiveProfile.Dispose();
            ActiveProfile = null;
        }

        if (m_pipeline != null)
        {
            m_pipeline.Dispose();
            m_pipeline = null;
        }
    }

    private void RaiseSampleEvent(FrameSet frames)
    {
        var onNewSample = OnNewSample;
        if (onNewSample != null)
        {
            onNewSample(frames);
        }
    }
    private void WaitForFrames()
    {
        while (!stopEvent.WaitOne(10))
        {
            //UnityEngine.Debug.Log(sync.Queue.Capacity);
            using (FrameSet frames = sync.WaitForFrames())
            {
                if (frames.Count == 1) {
                    Debug.Log("!!!");
                    //Debug.Log("dropped frame : " + frames + " " + frames.Profile.Stream + " " + frames.Timestamp + " " + frames.Number);
                }
                //Debug.Log(frames.Count);
                if (frames.Count == 2)
                {
                    //Debug.Log("!!!!!");
                    //Debug.Log( frames + " "  + frames.Timestamp);
                    RaiseSampleEvent(frames);
                    // Debug.Log(frames.DepthFrame.Width + " " + frames.DepthFrame.Height);
                    // Debug.Log(frames.ColorFrame.Width + " " + frames.ColorFrame.Height);
                    //pointCloudAttempt(frames);
                }
            }
        }
    }
    private void pointCloudAttempt(FrameSet frames) {
        using (var something = pb.Process(frames)) {
            UnityEngine.Debug.Log(something);
            UnityEngine.Debug.Log(something.AsFrameSet().Count);
            // using (var fs = something.As<FrameSet>())
            // using (var points = fs.FirstOrDefault<Points>(Stream.Depth, Format.Xyz32f)){
            //     points.ExportToPLY("C:/Users/Demo/Desktop/pointcloud2.ply", fs.DepthFrame);
            //     points.Dispose();
            // }
            //RaiseSampleEvent(something.AsFrameSet());
        }
        //Debug.Log(pb); don't dispore of points
        
    }

    // Update is called once per frame
    void Update()
    {
        if (!Streaming)
            return;

        FrameSet frames;
        if (sync.PollForFrames(out frames))
        {
            using (frames){
                if (frames.Count == 2) {
                    RaiseSampleEvent(frames);
                    //rosConnector.unsub = true;
                    //Debug.Log(frames.DepthFrame.Width + " " + frames.DepthFrame.Height);
                    //Debug.Log(frames.ColorFrame.Width + " " + frames.ColorFrame.Height);
                }
            }
        }
    }
}
