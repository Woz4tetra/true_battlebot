public class Velocity2d
{
    public float vx { get; set; }  // m/s
    public float vy { get; set; }  // m/s
    public float vyaw { get; set; }  // rad/s
    public Velocity2d()
    {
        vx = 0.0f;
        vy = 0.0f;
    }

    public Velocity2d(float vx, float vy)
    {
        this.vx = vx;
        this.vy = vy;
        vyaw = 0.0f;
    }

    public Velocity2d(float vx, float vy, float vyaw)
    {
        this.vx = vx;
        this.vy = vy;
        this.vyaw = vyaw;
    }
}
