using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Intel.RealSense;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Protocols;
using std_msgs = RosSharp.RosBridgeClient.MessageTypes.Std;
using sensor_msgs = RosSharp.RosBridgeClient.MessageTypes.Sensor;
using System.Net.Sockets;
using System.Threading;
using UnityEditorInternal;
public class RosBridge : MonoBehaviour
{
    private int SecondsTimeout = 5000;
    public RosSocket rosSocket;
    public ManualResetEvent IsConnected { get; private set; }
    public string RosBridgeServerUrl = "ws://169.254.192.1:9090";
    public Protocol protocolType;
    public RosSocket.SerializerEnum Serializer;

    private string depthImageTopic = "/camera/aligned_depth_to_color/image_raw_throttle";
    //private string depthImageTopic = "/camera/aligned_depth_to_color/image_raw";
    //private string depthImageTopic = "/camera/aligned_depth_to_color/image_raw/compressedDepth";
    private string depthImageTopicId;
    private string depthIntrinsicTopic = "/camera/aligned_depth_to_color/camera_info";
    private string depthIntrinsicId;
    private string depthImage;
    private Intrinsics depthIntrinsics;

    //private string colorImageTopic = "/camera/color/image_raw";
    private string colorImageTopic = "/camera/color/image_raw_throttle";
    //private string colorImageTopic = "/camera/color/image_raw/compressed";
    private string colorImageTopicId;
    private string colorIntrinsicTopic = "/camera/color/camera_info"; //don't use this one its the same as the other 
    private string colorIntrinsicId;
    private string colorImage;
    private Intrinsics colorIntrinsics;
    private ushort[] depthData;
    public bool depthIntrinsicsSet;
    public bool colorIntrinsicsSet;

    private int depthCounter;
    private int colorCounter;
    private int topicCounter;


    public Queue<RosSharp.RosBridgeClient.MessageTypes.Sensor.Image> depthQueue = 
        new Queue<RosSharp.RosBridgeClient.MessageTypes.Sensor.Image>();

    public Queue<RosSharp.RosBridgeClient.MessageTypes.Sensor.Image> colorQueue =
        new Queue<RosSharp.RosBridgeClient.MessageTypes.Sensor.Image>();

    SoftwareDev realsense;
    public bool unsub;

    // Start is called before the first frame update
    void Awake()
    {
        depthIntrinsicsSet = false;
        colorIntrinsicsSet = false;
        depthCounter = 0;
        colorCounter = 0;
        unsub = false;
        //Init msg
        UnityEngine.Debug.Log("Initializing WebSocket connection");
        IsConnected = new ManualResetEvent(false);

        //WebSocket Connection
        IProtocol protocol = ProtocolInitializer.GetProtocol(protocolType, RosBridgeServerUrl);
        rosSocket = new RosSocket(protocol, Serializer);
        IsConnected.Set();

        if (!IsConnected.WaitOne(SecondsTimeout * 1))
                Debug.LogWarning("Failed to connect to RosBridge at: " + RosBridgeServerUrl);

        //want to have depth/color intrinsics ready to be grabbed by the software device
        depthIntrinsicId = rosSocket.Subscribe<RosSharp.RosBridgeClient.MessageTypes.Sensor.CameraInfo>(depthIntrinsicTopic, depthIntrinsicHandler);
        colorIntrinsicId = rosSocket.Subscribe<RosSharp.RosBridgeClient.MessageTypes.Sensor.CameraInfo>(colorIntrinsicTopic, colorIntrinsicHandler);

        //we want to synch up the depth and color image topics and have that be what the pipeline
        //goes to look at when examining it's streams
        depthImage = rosSocket.Subscribe<RosSharp.RosBridgeClient.MessageTypes.Sensor.Image>(depthImageTopic, depthImageHandler);
        colorImage = rosSocket.Subscribe<RosSharp.RosBridgeClient.MessageTypes.Sensor.Image>(colorImageTopic, colorImageHandler);
        //depthImage = rosSocket.Subscribe<RosSharp.RosBridgeClient.MessageTypes.Sensor.CompressedImage>(depthImageTopic, depthImageHandler);
        //colorImage = rosSocket.Subscribe<RosSharp.RosBridgeClient.MessageTypes.Sensor.CompressedImage>(colorImageTopic, colorImageHandler);
        Debug.Log("subscribed!");
        realsense = GameObject.Find("RsDevice").gameObject.GetComponent<SoftwareDev>();
        realsense.OnStop += onStop;
        realsense.Invoke("onEnable", 0);
        //check buffer size 
        IsConnected.Set();
    }

    public Intrinsics getDepthCameraIntrinsics()
    {
        return depthIntrinsics;
    }

    public Intrinsics getColorCameraIntrinsics()
    {
        return colorIntrinsics;
    }
    private void depthIntrinsicHandler(sensor_msgs.CameraInfo depthInfo)
    {
        
        float[] coeff = new float[depthInfo.D.Length];
        for(int i = 0; i < depthInfo.D.Length; i++)
        {
            coeff[i] = (float)depthInfo.D[i];
        }
        depthIntrinsics = new Intrinsics {
            width = (int)depthInfo.width,
            height = (int)depthInfo.height,
            ppx = (float)depthInfo.K[2],
            ppy = (float)depthInfo.K[5],
            fx = (float)depthInfo.K[0],
            fy = (float)depthInfo.K[4],
            model = 0, // I don't know which one is plumb_bob
            coeffs = coeff
        };
        depthIntrinsicsSet = true;
        rosSocket.Unsubscribe(depthIntrinsicId);
    }

    private void colorIntrinsicHandler(sensor_msgs.CameraInfo colorInfo)
    {
        float[] coeff = new float[colorInfo.D.Length];
        for (int i = 0; i < colorInfo.D.Length; i++)
        {
            coeff[i] = (float) colorInfo.D[i];
        }
        colorIntrinsics = new Intrinsics
        {
            width = (int)colorInfo.width,
            height = (int)colorInfo.height,
            ppx = (float)colorInfo.K[2],
            ppy = (float)colorInfo.K[5],
            fx = (float)colorInfo.K[0],
            fy = (float)colorInfo.K[4],
            model = 0, // I don't know which one is plumb_bob
            coeffs = coeff
        };
        colorIntrinsicsSet = true;
        rosSocket.Unsubscribe(colorIntrinsicId);
        topicCounter++;
    }

    private void depthImageHandler(sensor_msgs.Image depth)
    {
        
        realsense.depth_sensor.AddVideoFrame<byte>(depth.data, (int) (depth.step), (int) (depth.step/ depth.width), (double)((depth.header.stamp.secs) * 1000 + (depth.header.stamp.nsecs / 1000000)), 0, (int) depth.header.seq, realsense.depthProfile);
        depthCounter++;

        //Debug.Log("rosbridge depth " + (double)((depth.header.stamp.secs) * 1000 + (depth.header.stamp.nsecs / 1000000))+ " " + (int) depth.header.seq);
    }

    private void colorImageHandler(sensor_msgs.Image color)
    {
        realsense.color_sensor.AddVideoFrame<byte>(color.data, (int) (color.step), (int) (color.step/ color.width), (double) ((color.header.stamp.secs * 1000) + (color.header.stamp.nsecs / 1000000)), 0, (int) color.header.seq, realsense.colorProfile);
        colorCounter++;
        //Debug.Log("rosbridge color " + (double) ((color.header.stamp.secs * 1000) + (color.header.stamp.nsecs / 1000000)) + " " + (int) color.header.seq);
    }
    //rostopic info should tell me the info 
    // lowering the resolution 
    void Update()
    {
        Debug.Log("DepthFPS: " + (depthCounter/Time.time)); //Time.time is in seconds
        Debug.Log("ColorFPS: " + (colorCounter/Time.time)); //Time.time is in seconds
        //Debug.Log(topicCounter / Time.time); // also comes out to around to 5-6
    }

    private void onStop() {
        rosSocket.Unsubscribe(depthImage);
        rosSocket.Unsubscribe(colorImage);
    }
}
