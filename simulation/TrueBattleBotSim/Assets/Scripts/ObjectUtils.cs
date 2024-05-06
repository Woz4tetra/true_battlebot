using UnityEngine;

public class ObjectUtils
{
    public static T GetComponentInTree<T>(GameObject obj) where T : Component
    {
        Transform tf = obj.transform;
        T component = obj.GetComponent<T>();
        while (true)
        {
            if (component != null)
            {
                break;
            }
            if (tf.parent == null)
            {
                break;
            }
            tf = tf.parent;
            obj = tf.gameObject;
            component = obj.GetComponent<T>();
        }
        return component;
    }

    public static GameObject GetTopLevelObject(GameObject obj)
    {
        Transform tf = obj.transform;
        while (true)
        {
            if (tf.parent == null)
            {
                break;
            }
            tf = tf.parent;
        }
        return tf.gameObject;
    }

}