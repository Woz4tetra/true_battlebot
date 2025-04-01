using UnityEngine;

namespace MathExtensions
{
    public static class MathfEx
    {
        public static float Wrap(float x, float m)
        {
            return (x % m + m) % m;
        }

        public static float NearestMultiple(float x, float f)
        {
            return Mathf.RoundToInt(x / f) * f;
        }

        public static float HigherMultiple(float x, float f)
        {
            return Mathf.CeilToInt(x / f) * f;
        }

        public static Vector2 Bezier(Vector2 s, Vector2 e, Vector2 st, Vector2 et, float t)
        {
            return (((-s + 3 * (st - et) + e) * t + (3 * (s + et) - 6 * st)) * t + 3 * (st - s)) * t + s;
        }

        public static Vector3 Bezier(Vector3 s, Vector3 e, Vector3 st, Vector3 et, float t)
        {
            return (((-s + 3 * (st - et) + e) * t + (3 * (s + et) - 6 * st)) * t + 3 * (st - s)) * t + s;
        }

        public static Matrix4x4 ScaleAroundPivot(Vector3 pivot, Vector3 scale)
        {
            return Matrix4x4.TRS(pivot, Quaternion.identity, scale) * Matrix4x4.TRS(-pivot, Quaternion.identity, Vector3.one);
        }

        public static Vector3 GetT(this Matrix4x4 trs)
        {
            return trs.GetColumn(3);
        }

        public static Quaternion GetR(this Matrix4x4 trs)
        {
            return Quaternion.LookRotation(trs.GetColumn(2), trs.GetColumn(1));
        }

        public static Vector3 GetS(this Matrix4x4 trs)
        {
            return new Vector3(trs.GetColumn(0).magnitude, trs.GetColumn(1).magnitude, trs.GetColumn(2).magnitude);
        }

        public static Matrix4x4 GetMatrix4x4(this Transform tf)
        {
            return Matrix4x4.TRS(tf.position, tf.rotation, tf.localScale);
        }

        public static Quaternion FromFLUEulerAngles(this Vector3 euler)
        {
            Quaternion quat = Quaternion.Euler(euler.x, euler.y, euler.z);
            return new Quaternion(-quat.x, quat.y, quat.z, quat.w);
        }
        public static Quaternion ToRUFAngles(this Vector3 euler)
        {
            return Quaternion.Euler(euler.y, -euler.z, euler.x);
        }

        public static Vector3 ToFLUEulerAngles(this Quaternion quat)
        {
            Quaternion q = new Quaternion(quat.z, -quat.x, quat.y, -quat.w);
            return q.eulerAngles;
        }


        /**
        Bound the number between min_value and max_value, wrapping around if it goes over.

        Examples:
            input_modulus(1, -1, 3) == 1
            input_modulus(6, -1, 3) == 2
            input_modulus(0, -1, 3) == 0
            input_modulus(5, -1, 3) == 1
        */
        public static float InputModulus(float value, float min_value, float max_value)
        {
            float modulus = max_value - min_value;

            value -= min_value;
            value = Mathf.Repeat(value, modulus);
            value += min_value;

            return value;
        }


        public static float NormalizeAnglePi(float angle)
        {
            return InputModulus(angle, -Mathf.PI, Mathf.PI);
        }
    }
}
