#include <updown_sensor.h>

using namespace updown_sensor;

UpdownSensor::UpdownSensor()
{
    sensor = new Adafruit_BNO055(55, 0x28, &Wire1);
    grav_vec = make_unit_vector(0.0, 0.0, -9.81);
    max_grav_vec = init_vector3(0.0, 0.0, 0.0);
    min_grav_vec = init_vector3(0.0, 0.0, 0.0);
    orientation = init_vector3(0.0, 0.0, 0.0);
    gyro_vec = init_vector3(0.0, 0.0, 0.0);
}

bool UpdownSensor::begin()
{
    if (initialized)
        return true;
    if (sensor->begin())
    {
        delay(1000);
        initialized = true;
        sensor->setExtCrystalUse(true);
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
    if (!update_sensor(radio_connected))
        return is_upside_down;

    float z = -1 * grav_vec->z;

    if (z < RIGHT_SIDE_UP_THRESHOLD)
        is_upside_down = false;
    else if (z > UPSIDE_DOWN_THRESHOLD)
        is_upside_down = true;
    return is_upside_down;
}

bool UpdownSensor::update_sensor(bool radio_connected)
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
    uint32_t now = millis();
    if (now - sample_timer < SAMPLE_INTERVAL)
    {
        return false;
    }
    sample_timer = now;
    uint32_t start_time = now;
    sensors_event_t gravity_data, orientation_data, gyro_data;
    sensor->getEvent(&gravity_data, Adafruit_BNO055::VECTOR_GRAVITY);
    sensor->getEvent(&orientation_data, Adafruit_BNO055::VECTOR_EULER);
    sensor->getEvent(&gyro_data, Adafruit_BNO055::VECTOR_GYROSCOPE);
    uint32_t end_time = millis();

    if (end_time - start_time > 250)
    {
        initialized = false;
        return false;
    }

    grav_vec->x = gravity_data.acceleration.x;
    grav_vec->y = gravity_data.acceleration.y;
    grav_vec->z = gravity_data.acceleration.z;

    max_grav_vec->x = max(max_grav_vec->x, grav_vec->x);
    max_grav_vec->y = max(max_grav_vec->y, grav_vec->y);
    max_grav_vec->z = max(max_grav_vec->z, grav_vec->z);

    min_grav_vec->x = min(min_grav_vec->x, grav_vec->x);
    min_grav_vec->y = min(min_grav_vec->y, grav_vec->y);
    min_grav_vec->z = min(min_grav_vec->z, grav_vec->z);

    orientation->x = orientation_data.orientation.x;
    orientation->y = orientation_data.orientation.y;
    orientation->z = orientation_data.orientation.z;

    gyro_vec->x = gyro_data.gyro.x;
    gyro_vec->y = gyro_data.gyro.y;
    gyro_vec->z = gyro_data.gyro.z;

    return true;
}