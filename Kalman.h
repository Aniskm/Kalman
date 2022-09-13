/* A simplified one dimensional Kalman filter implementation - actually a single variable low pass filter ;-)
   Based on: http://interactive-matter.eu/blog/2009/12/18/filtering-sensor-data-with-a-kalman-filter/
*/

/*
q = process noise covariance   
r = measurement noise covariance
x = value of interest
p = estimation error covariance
k = Kalman gain
---------------
H = 1 =>  we will assume H, O/ are constant ( for linear filters they are )

*/

#ifndef _Kalman_h
#define _Kalman_h

class Kalman {
  private:
    /* Kalman filter variables */
    double q; //process noise covariance
    double r; //measurement noise covariance
    double x; //value
    double p; //estimation error covariance
    double k; //kalman gain
    
  public:
    Kalman(double process_noise, double sensor_noise, double estimated_error, double intial_value) {
      /* The variables are x for the filtered value, q for the process noise, 
         r for the sensor noise, p for the estimated error and k for the Kalman Gain. 
         The state of the filter is defined by the values of these variables.
         
         The initial values for p is not very important since it is adjusted
         during the process. It must be just high enough to narrow down.
         The initial value for the readout is also not very important, since
         it is updated during the process.
         But tweaking the values for the process noise and sensor noise
         is essential to get clear readouts.
         
         For large noise reduction, you can try to start from: (see http://interactive-matter.eu/blog/2009/12/18/filtering-sensor-data-with-a-kalman-filter/ )
         q = 0.125
         r = 32
         p = 1023 //"large enough to narrow down"
         e.g.
         myVar = Kalman(0.125,32,1023,0);
      */
        this->q = process_noise;
        this->r = sensor_noise;
        this->p = estimated_error;
        this->x = intial_value; //x will hold the iterated filtered value
    }
    
    double getFilteredValue(double measurement) {
      /* Updates and gets the current measurement value */
      //prediction update
      //omit x = x
      this->p = this->p + this->q;
    
      //measurement update
       // K = Kalman Gain |  P = Schätzfehler |  p+r = Gesamtfehler | r = Fehler der Messung 
       // 1 - K => Gegenstück von Kalman Gain = r/(p+r)
      this->k = this->p / (this->p + this->r); // K = (P* H) /( H * P * H + R)   Update Kalman Gain  
      // K = Schätzfehler(p) / ( Schätzfehler(p) + Fehler der Messung (r)
       this->x = this->x + this->k * (measurement - this->x); // U` + = K* (U-H * U') update estimate x= U dach H = 1 U = measurement 
       // Schätz(t) = Schätz(t-1) + K*(Mess(t)- Schätz(t-1))
      this->p = (1 - this->k) * this->p; // P = (1- K * H ) * P + Q ich gehe davon aus H = =1 und q = 0 Update error estimate 
      // aktuale schätze Fehler => F Schätz (p) = (1-K)* F schätz(t-1) ( alter Schätzfehler)
      return this->x; 
    }
    
    void setParameters(double process_noise, double sensor_noise, double estimated_error) {
        this->q = process_noise;
        this->r = sensor_noise;
        this->p = estimated_error;
    }

    void setParameters(double process_noise, double sensor_noise) {
        this->q = process_noise;
        this->r = sensor_noise;
    }
    
    double getProcessNoise() {
      return this->q;
    }
    
    double getSensorNoise() {
      return this->r;
    }
    
    double getEstimatedError() {
      return this->p;
    }
};

#endif
