using RosMessageTypes.BwInterfaces;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class CollisionSensor : MonoBehaviour
{
    [SerializeField] private string collisionInfoTopic = "simulation/collision_info";
    ROSConnection ros;
    RosTopicState topicState;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        topicState = ros.GetOrCreateTopic(collisionInfoTopic, MessageRegistry.GetRosMessageName<CollisionInfoMsg>());
        if (!topicState.IsPublisher)
        {
            ros.RegisterPublisher<CollisionInfoMsg>(topicState.Topic);
        }
    }

    void OnTriggerEnter(Collider other)
    {
        CollisionCallback(other);
    }

    void OnCollisionEnter(Collision collision)
    {
        CollisionCallback(collision.collider);
    }

    void CollisionCallback(Collider other)
    {
        if (ros == null) return;
        GameObject topLevelObject = ObjectUtils.GetTopLevelObject(other.gameObject);
        TransformFrame otherFrame = topLevelObject.GetComponent<TransformFrame>();
        string otherName = otherFrame != null ? otherFrame.GetFrameId() : topLevelObject.name;
        TransformFrame thisFrame = GetComponent<TransformFrame>();
        string thisName = thisFrame != null ? thisFrame.GetFrameId() : gameObject.name;
        CollisionInfoMsg msg = new CollisionInfoMsg
        {
            source_object = thisName,
            collision_with = otherName
        };
        topicState.Publish(msg);
    }
}