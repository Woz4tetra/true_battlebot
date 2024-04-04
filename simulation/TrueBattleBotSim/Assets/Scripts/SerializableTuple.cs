using System;
using System.Collections;
using UnityEngine;

[Serializable]
public class SerializableTuple<T1, T2> : IStructuralComparable, IStructuralEquatable, IComparable
{
    [SerializeField]
    private T1 item1;

    [SerializeField]
    private T2 item2;

    public T1 Item1 => item1;
    public T2 Item2 => item2;

    public SerializableTuple(T1 item1, T2 item2)
    {
        this.item1 = item1;
        this.item2 = item2;
    }

    public int CompareTo(object other, IComparer comparer)
    {
        throw new NotImplementedException(); // TODO
    }

    public int CompareTo(object obj)
    {
        throw new NotImplementedException(); // TODO
    }

    public bool Equals(object other, IEqualityComparer comparer)
    {
        throw new NotImplementedException(); // TODO
    }

    public int GetHashCode(IEqualityComparer comparer)
    {
        throw new NotImplementedException(); // TODO
    }
}
