using System;
using UnityEngine;
using Intel.RealSense;
using UnityEngine.Rendering;
using UnityEngine.Assertions;
using System.Runtime.InteropServices;
using System.Threading;
using System.Collections.Generic;
using System.Linq;
using Intel.RealSense.Base;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class RsPointCloudRenderer : MonoBehaviour
{
    public RsFrameProvider Source;
    private Mesh mesh;
    private Texture2D uvmap;

    [NonSerialized]
    private Vector3[] vertices;

    FrameQueue q;

    void Start()
    {
        Source.OnStart += OnStartStreaming;
        Source.OnStop += Dispose;
    }

    private void OnStartStreaming(PipelineProfile obj)
    {
        UnityEngine.Debug.Log("start streaming");
        q = new FrameQueue(1);

        using (var depth = obj.Device.Sensors[0].StreamProfiles[0].As<VideoStreamProfile>())
            ResetMesh(depth.Width, depth.Height);
        //ResetMesh(640, 480);

        Source.OnNewSample += OnNewSample;
    }

    private void ResetMesh(int width, int height)
    {
        Assert.IsTrue(SystemInfo.SupportsTextureFormat(TextureFormat.RGFloat));
        uvmap = new Texture2D(width, height, TextureFormat.RGFloat, false, true)
        {
            wrapMode = TextureWrapMode.Clamp,
            filterMode = FilterMode.Point,
        };
        GetComponent<MeshRenderer>().sharedMaterial.SetTexture("_UVMap", uvmap);

        if (mesh != null)
            mesh.Clear();
        else
            mesh = new Mesh()
            {
                indexFormat = IndexFormat.UInt32,
            };

        vertices = new Vector3[width * height];

        var indices = new int[vertices.Length];
        for (int i = 0; i < vertices.Length; i++)
            indices[i] = i;

        mesh.MarkDynamic();
        mesh.vertices = vertices;

        var uvs = new Vector2[width * height];
        Array.Clear(uvs, 0, uvs.Length);
        for (int j = 0; j < height; j++)
        {
            for (int i = 0; i < width; i++)
            {
                uvs[i + j * width].x = i / (float)width;
                uvs[i + j * width].y = j / (float)height;
            }
        }

        mesh.uv = uvs;

        mesh.SetIndices(indices, MeshTopology.Points, 0, false);
        mesh.bounds = new Bounds(Vector3.zero, Vector3.one * 10f);

        GetComponent<MeshFilter>().sharedMesh = mesh;
    }

    void OnDestroy()
    {
        if (q != null)
        {
            q.Dispose();
            q = null;
        }

        if (mesh != null)
            Destroy(null);
    }

    private void Dispose()
    {
        Source.OnNewSample -= OnNewSample;

        if (q != null)
        {
            q.Dispose();
            q = null;
        }
    }

    private void OnNewSample(Frame frame)
    {
        if (q == null)
            return;
        try
        {
            if (frame.IsComposite)
            {
                using (var fs = frame.As<FrameSet>())
                using (var points = fs.FirstOrDefault<Points>(Stream.Depth, Format.Xyz32f))
                {
                    if (points != null)
                    {
                        //points.ExportToPLY("C:/Users/Demo/Desktop/pointcloud1.ply", fs.DepthFrame);
                        q.Enqueue(points);
                    }
                }
                return;
            }

            if (frame.Is(Extension.Points))
            {
                q.Enqueue(frame);
            }
        }
        catch (Exception e)
        {
            Debug.LogException(e);
        }
    }


    
    protected void LateUpdate()
    {
        if (q != null)
        {
            Points points;
            if (q.PollForFrame<Points>(out points))
                using (points)
                {
                    if (points.Count != mesh.vertexCount)
                    {
                        using (var p = points.GetProfile<VideoStreamProfile>()){
                            UnityEngine.Debug.Log(p.Width + " " + p.Height);
                            ResetMesh(p.Width, p.Height);
                        }
                    }

                    if (points.TextureData != IntPtr.Zero)
                    {
                        //UnityEngine.Debug.Log(points.TextureData + " " + points.Count);
                        uvmap.LoadRawTextureData(points.TextureData, points.Count * sizeof(float) * 2);
                        uvmap.Apply();
                        //points.ExportToPLY("C:/Users/Demo/Desktop/pointcloud1.ply", new VideoFrame(points.TextureData));
                       // UnityEngine.Debug.Log("applied");
                    }

                    if (points.VertexData != IntPtr.Zero)
                    {
                        points.CopyVertices(vertices);

                        mesh.vertices = vertices;
                        mesh.UploadMeshData(false);
                        //UnityEngine.Debug.Log("applied applied");
                    }
                }
        }
    }
}
