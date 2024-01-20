#include <iostream>
#include <eigen3/Eigen/Dense>



int main()
{
    
    
    Eigen::Vector3d _gamma;
    Eigen::Vector3d _Power_in;
    
    double _P_up = 4.0;
    double _P_low = -4.0;

    _gamma << 2,4,6;
    _Power_in<< 1.0, -0.1, -0.3;
    std::cout << _gamma.transpose()<<std::endl;

    for(int i=0; i<3; i++)
    {
        if (_Power_in(i) < 0){
            _gamma(i) = 0;

        }
        else{_gamma(i) = _gamma(i); }
    }

    std::cout << _gamma.transpose()<<std::endl;

    for(int i=0; i<3; i++)
    {
        if (_gamma(i)*_Power_in(i) > _P_up){
            _gamma(0) = _P_up/_Power_in(0);
        }
        else if (_gamma(i)*_Power_in(i) <_P_low){
            _gamma(i) = _P_low/_Power_in(i);
        }
        else{_gamma(i) = _gamma(i);}
    }

    // if(_gamma(0)*_Power_in(0) >_P_up){_gamma(0) = _P_up/_Power_in(0);} else if(_gamma(0)*_Power_in(0) <_P_low){_gamma(0) = _P_low/_Power_in(0);} else {_gamma(0) = _gamma(0); };
	// if(_gamma(1)*_Power_in(1) >_P_up){_gamma(1) = _P_up/_Power_in(1);} else if(_gamma(1)*_Power_in(1) <_P_low){_gamma(1) = _P_low/_Power_in(1);} else {_gamma(1) = _gamma(1); };
	// if(_gamma(2)*_Power_in(2) >_P_up){_gamma(2) = _P_up/_Power_in(2);} else if(_gamma(2)*_Power_in(2) <_P_low){_gamma(2) = _P_low/_Power_in(2);} else {_gamma(2) = _gamma(2); };
    if(_gamma(0)*_Power_in(0) >_P_up){_gamma(0) = _gamma(0)*_P_up/_P_total;} else if(_gamma(0)*_Power_in(0) <_P_low){_gamma(0) =_gamma(0)* _P_low/_P_total;} else {_gamma(0) = _gamma(0); };
    if(_gamma(1)*_Power_in(1) >_P_up){_gamma(1) = _gamma(1)*_P_up/_P_total;} else if(_gamma(1)*_Power_in(1) <_P_low){_gamma(1) =_gamma(1)* _P_low/_P_total;} else {_gamma(1) = _gamma(1); };
    if(_gamma(2)*_Power_in(2) >_P_up){_gamma(2) = _gamma(2)*_P_up/_P_total;} else if(_gamma(2)*_Power_in(2) <_P_low){_gamma(2) =_gamma(2)* _P_low/_P_total;} else {_gamma(2) = _gamma(2); };

    std::cout << _gamma.transpose()<<std::endl;
    return 0;
}