#ifndef THRUSTER_H
#define THRUSTER_H
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
    
        //it sets the struct values of any instance of this class
        void setValues(int HC, int MC,int LC, int HAC, int MAC, int LAC);
    
        //it classifies the obtained pwm variable to given specified speed modes
        void calibration(int PWM, int M);

	private:
        int pwm_;        //stores the value of pwm corresponding to each thruster
        mode clock_;     //stores the value of high,med and low pwm values for clockwise direction
        mode anticlock_;    //stores the value of high,med and low pwm values for anticlockwise direction
};

#endif


