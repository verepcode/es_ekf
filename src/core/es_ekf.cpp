
class StateEstimator {

    public:
        StateEstimator();
        ~StateEstimator();

        void es_ekf(sensor_msgs::Imu imu_data, gnss_data);

    private:

    

}