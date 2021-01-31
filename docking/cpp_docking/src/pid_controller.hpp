
class PIDController{
    public:
        PIDController(float dt);
        ~PIDController();
        float* getVelocities(float x_err,float y_err, float alt_err, float max_speed);
    private:
        float m_dt;
        float* m_prev_errs;
};