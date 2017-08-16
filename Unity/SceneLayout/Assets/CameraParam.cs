using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraParam : MonoBehaviour {
    [ContextMenu("Copy Camera Param")]
    void CopyCameraParam()
    {
        var p = this.transform.position;
        var r = this.transform.rotation;
        var s = this.transform.lossyScale;
        var lookat = this.transform.forward + p;
        var up = this.transform.up;

        string src = "";
        src += string.Format("cameraSetting._eye = Vec3({0}, {1}, {2});\n", p.x, p.y, -p.z);
        src += string.Format("cameraSetting._lookat = Vec3({0}, {1}, {2});\n", lookat.x, lookat.y, -lookat.z);
        src += string.Format("cameraSetting._up = Vec3({0}, {1}, {2});\n", up.x, up.y, -up.z);
        src += string.Format("cameraSetting._fov = {0};\n", Mathf.Deg2Rad * GetComponent<Camera>().fieldOfView);

        GUIUtility.systemCopyBuffer = src;
    }
}
