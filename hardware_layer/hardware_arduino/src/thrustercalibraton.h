#ifndef THRUSTERCALIBRATION_H
#define THRUSTERCALIBRATION_H
using namespace std;

struct mode         //structure to store the three speed mode values in one direction
{
    int High;
    int Med;
    int Low;
};

class thruster
{
	public:
    
		thruster();
    
        //tells the speed mode of the thruster
        int mode(int P);
    
        //it classifies the obtained pwm variable to given specified speed modes
        void calibration(int PWM, int M);
    
        //it sets the struct values of any instance of this class
        void setvalues(int HC, int MC,int LC, int HAC, int MAC, int LAC);

	private:
        int pwm_;        //stores the value of pwm corresponding to each thruster
        mode clock_;     //stores the value of high,med and low pwm values for clockwise direction
        mode anticlock_;    //stores the value of high,med and low pwm values for anticlockwise direction
};

class arduino_thrust
{
    public:
    
        arduino_thrust();
    
        //sets the direction and pwm pins for the thrusters and initiate them as outputs
        void setPins(int A1, int D1, int D2);
    
        //applies analogwrite and digitalwrite functions in arduino
        void ON(int pwm,bool Dir1,bool Dir2);
    
    private:
        int An1_;   //stores the analog pin number for the thruster on arduino
        int D1_;    //stores the direction pin number 1 for the thruster on arduino
        int D2_;    //stores the direction pin number 2 for the thruster on arduino
    
};
#endif


