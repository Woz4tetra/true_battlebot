using UnityEngine;

public class ObjectUtils
{
    public static T GetComponentInTree<T>(GameObject obj)
    {
        Transform tf = obj.transform;
        T component = obj.GetComponent<T>();
        while (true)
        {
            bool should_break = component is not null;

            if (Application.isEditor)
            {
                // ???somehow this only works in reverse for editor only???
                should_break = !should_break;
            }
            if (should_break)
            {
                break;
            }
            if (tf.parent is null)
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