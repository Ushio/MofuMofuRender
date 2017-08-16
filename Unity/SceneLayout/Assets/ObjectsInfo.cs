using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class ObjectsInfo : MonoBehaviour {
    [ContextMenu("Copy Objects Param")]
    void CopyObjectsParam()
    {
        string src = "";

        for(int i = 0; i < transform.childCount; ++i)
        {
            var child = transform.GetChild(i);
            var mr = child.GetComponent<MeshRenderer>();
            var mf = child.GetComponent<MeshFilter>();
            var mesh = mf.sharedMesh;
            var vs = mesh.vertices;
            var indices = mesh.GetIndices(0);

            src += "	{\n";
            src += "		std::vector<Vec3> vertices = {\n";

            src += "		" + string.Join(",\n", vs.Select(p => string.Format("rt::Vec3({0}, {1}, {2})", p.x, p.y, p.z)).ToArray());

            src += "		};\n";
            src += "		std::vector<int> indices = {" + string.Join(",", indices.Select(index => index.ToString()).ToArray()) + "};\n";

            src += "\n";

            var pos = child.position;
            var r = child.rotation;
            var s = child.lossyScale;
            float angle = 0.0f;
            Vector3 axis = Vector3.zero;
            r.ToAngleAxis(out angle, out axis);

            string transform_src = string.Format("Transform(Vec3({0}, {1}, {2}), glm::angleAxis(double({3}), Vec3({4}, {5}, {6})), Vec3({7}, {8}, {9}));",
                pos.x, pos.y, -pos.z,
                -Mathf.Deg2Rad * angle, axis.x, axis.y, -axis.z,
                s.x, s.y, s.z
            );

            src += "		Transform transform = " + transform_src + "\n";

            src += "		transform.apply(vertices);\n";
            src += "		std::shared_ptr<rt::PolygonSceneElement> element(new rt::PolygonSceneElement(vertices, indices,\n";

            var albedo = mr.sharedMaterial.color;
            src += string.Format("			LambertianMaterial(rt::Vec3(0.0), rt::Vec3({0}, {1}, {2})),\n", albedo.r, albedo.g, albedo.b);

            src += "			false\n";
            src += "		));\n";
            src += "		sceneElements.push_back(element);\n";
            src += "	}\n";
        }

        GUIUtility.systemCopyBuffer = src;
    }
}
