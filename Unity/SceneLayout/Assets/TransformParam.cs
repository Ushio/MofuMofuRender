using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TransformParam : MonoBehaviour {
    [ContextMenu("Copy Transform Param")]
    void CopyTransformParam()
    {
        var p = this.transform.position;
        var r = this.transform.rotation;
        var s = this.transform.lossyScale;

        float angle = 0.0f;
        Vector3 axis = Vector3.zero;
        r.ToAngleAxis(out angle, out axis);
        string src = string.Format("Transform(Vec3({0}, {1}, {2}), glm::angleAxis({3}, Vec3({4}, {5}, {6})), Vec3({7}, {8}, {9}));",
            p.x, p.y, -p.z,
            -Mathf.Deg2Rad * angle, axis.x, axis.y, -axis.z,
            s.x, s.y, s.z
        );

        GUIUtility.systemCopyBuffer = src;
    }
}
