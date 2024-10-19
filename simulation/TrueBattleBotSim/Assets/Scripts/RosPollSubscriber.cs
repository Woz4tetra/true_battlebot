using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class QueueWithCapacity<T>
{
    private Queue<T> queue = new Queue<T>();
    private int capacity;
    public QueueWithCapacity(int capacity)
    {
        this.capacity = capacity;
    }

    public void Enqueue(T item)
    {
        while (queue.Count > capacity)
        {
            queue.Dequeue();
        }
        queue.Enqueue(item);
    }

    public T Dequeue()
    {
        return queue.Dequeue();
    }

    public int Count
    {
        get
        {
            return queue.Count;
        }
    }
}

public class RosPollSubscriber<T> where T : Message
{
    private ROSConnection ros;
    private QueueWithCapacity<T> messageQueue;
    public RosPollSubscriber(string topic, int queueSize = 1)
    {
        ros = ROSConnection.GetOrCreateInstance();
        messageQueue = new QueueWithCapacity<T>(queueSize);
        lock (messageQueue)
        {
            Debug.Log($"Subscribing to {topic}");
            ros.Subscribe<T>(topic, MessageCallback);
        }
    }

    private void MessageCallback(T message)
    {
        lock (messageQueue)
        {
            messageQueue.Enqueue(message);
        }
    }

    public Optional<T> Receive()
    {
        lock (messageQueue)
        {
            Optional<T> message;
            if (messageQueue.Count > 0)
            {
                message = Optional<T>.Create(messageQueue.Dequeue());
            }
            else
            {
                message = Optional<T>.CreateEmpty();
            }
            return message;
        }
    }
}
