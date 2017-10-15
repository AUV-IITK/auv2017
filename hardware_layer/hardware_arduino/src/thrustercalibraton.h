#ifndef PWM_CONST.H
#define PWM_CONST.H

struct mode         //structure to store the three speed mode values in one direction
{
    int High;
    int Med;
    int Low;
};

using namespace std;
class thruster{
	public:
		thruster();
		~thruster();
    
        //tells the speed mode of the thruster
        int mode();
    
        //it classifies the obtained pwm variable to given specified speed modes
        void calibration();
    
        //it sets the struct values of any instance of this class
        void setvalues();

	private:
        int pwm_;        //stores the value of pwm corresponding to each thruster
        mode clock_;     //stores the value of high,med and low pwm values for clockwise direction
        mode anticlock_;    //stores the value of high,med and low pwm values for anticlockwise direction
}

#endif


