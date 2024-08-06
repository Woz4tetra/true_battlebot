using System.Collections.Generic;

public class Optional<T> : IEnumerable<T>
{
    private readonly T[] data;

    private Optional(T[] data)
    {
        this.data = data;
    }

    public static Optional<T> Create(T value)
    {
        return new Optional<T>(new T[] { value });
    }

    public static Optional<T> CreateEmpty()
    {
        return new Optional<T>(new T[0]);
    }

    public IEnumerator<T> GetEnumerator()
    {
        return ((IEnumerable<T>)this.data).GetEnumerator();
    }

    public bool TryGet(out T value)
    {
        if (this.data.Length > 0)
        {
            value = this.data[0];
            return true;
        }
        value = default(T);
        return false;
    }

    System.Collections.IEnumerator
        System.Collections.IEnumerable.GetEnumerator()
    {
        return this.data.GetEnumerator();
    }
}