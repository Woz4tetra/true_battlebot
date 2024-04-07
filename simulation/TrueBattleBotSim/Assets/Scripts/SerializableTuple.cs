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
        if (other == null)
        {
            return 1;
        }
        if (!(other is SerializableTuple<T1, T2>))
        {
            throw new ArgumentException("Argument must be a SerializableTuple<T1, T2>", nameof(other));
        }
        SerializableTuple<T1, T2> otherTuple = (SerializableTuple<T1, T2>)other;
        int cmp = comparer.Compare(item1, otherTuple.item1);
        if (cmp != 0)
        {
            return cmp;
        }
        return comparer.Compare(item2, otherTuple.item2);
    }

    public int CompareTo(object obj)
    {
        if (obj == null)
        {
            return 1;
        }
        if (!(obj is SerializableTuple<T1, T2>))
        {
            throw new ArgumentException("Argument must be a SerializableTuple<T1, T2>", nameof(obj));
        }
        SerializableTuple<T1, T2> otherTuple = (SerializableTuple<T1, T2>)obj;
        int cmp = Comparer.Default.Compare(item1, otherTuple.item1);
        if (cmp != 0)
        {
            return cmp;
        }
        return Comparer.Default.Compare(item2, otherTuple.item2);
    }

    public bool Equals(object other, IEqualityComparer comparer)
    {
        if (other == null)
        {
            return false;
        }
        if (!(other is SerializableTuple<T1, T2>))
        {
            return false;
        }
        SerializableTuple<T1, T2> otherTuple = (SerializableTuple<T1, T2>)other;
        return comparer.Equals(item1, otherTuple.item1) && comparer.Equals(item2, otherTuple.item2);
    }

    public int GetHashCode(IEqualityComparer comparer)
    {
        return HashCode.Combine(comparer.GetHashCode(item1), comparer.GetHashCode(item2));
    }
}
