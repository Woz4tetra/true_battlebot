#include <updown_sensor.h>

using namespace updown_sensor;

UpdownSensor::UpdownSensor()
{
    accel = new Adafruit_ADXL375(12345);
    accel_vec = make_unit_vector(0.0, 0.0, -1.0);
    max_accel_vec = init_vector3(0.0, 0.0, 0.0);
    min_accel_vec = init_vector3(0.0, 0.0, 0.0);
}

bool UpdownSensor::begin()
{
    if (initialized)
        return true;
    if (accel->begin())
    {
        accel->setTrimOffsets(0, 0, 0);
        initialized = true;
        return true;
    }
    else
    {
        initialized = false;
        return false;
    }
}

vector3_t *UpdownSensor::make_unit_vector(float x, float y, float z)
{
    float magnitude = sqrt(x * x + y * y + z * z);
    vector3_t *unit_vector = (vector3_t *)malloc(sizeof(vector3_t));
    unit_vector->x = x / magnitude;
    unit_vector->y = y / magnitude;
    unit_vector->z = z / magnitude;
    return unit_vector;
}

vector3_t *UpdownSensor::init_vector3(float x, float y, float z)
{
    vector3_t *vec = (vector3_t *)malloc(sizeof(vector3_t));
    vec->x = x;
    vec->y = y;
    vec->z = z;
    return vec;
}

bool UpdownSensor::get_is_upside_down(bool radio_connected)
{
    if (!get_accel(radio_connected))
        return is_upside_down;

    float z = accel_vec->z;

    if (z > RIGHT_SIDE_UP_THRESHOLD)
        is_upside_down = false;
    else if (z < UPSIDE_DOWN_THRESHOLD)
        is_upside_down = true;
    return is_upside_down;
}

bool UpdownSensor::get_accel(bool radio_connected)
{
    if (!initialized && !radio_connected)
    {
        if (millis() - reconnect_timer > RECONNECT_INTERVAL)
        {
            begin();
            reconnect_timer = millis();
        }
        return false;
    }
    sensors_event_t event;
    uint32_t start_time = millis();
    accel->getEvent(&event);
    uint32_t end_time = millis();

    if (end_time - start_time > 250)
    {
        initialized = false;
        return false;
    }

    accel_vec->x = event.acceleration.x;
    accel_vec->y = event.acceleration.y;
    accel_vec->z = event.acceleration.z;

    max_accel_vec->x = max(max_accel_vec->x, accel_vec->x);
    max_accel_vec->y = max(max_accel_vec->y, accel_vec->y);
    max_accel_vec->z = max(max_accel_vec->z, accel_vec->z);

    min_accel_vec->x = min(min_accel_vec->x, accel_vec->x);
    min_accel_vec->y = min(min_accel_vec->y, accel_vec->y);
    min_accel_vec->z = min(min_accel_vec->z, accel_vec->z);

    return true;
}