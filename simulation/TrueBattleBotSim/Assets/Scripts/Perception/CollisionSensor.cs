using RosMessageTypes.BwInterfaces;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class CollisionSensor : MonoBehaviour
{
    [SerializeField] private string collisionInfoTopic = "simulation/collision_info";
    ROSConnection ros;
    private static bool registered = false;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        if (!registered)
        {
            ros.RegisterPublisher<CollisionInfoMsg>(collisionInfoTopic);
            registered = true;
        }

    }
    void OnTriggerEnter(Collider other)
    {
        GameObject topLevelObject = ObjectUtils.GetTopLevelObject(other.gameObject);
        CollisionInfoMsg msg = new CollisionInfoMsg
        {
            source_object = gameObject.name,
            collision_with = topLevelObject.name
        };
        ros.Publish(collisionInfoTopic, msg);
    }
}