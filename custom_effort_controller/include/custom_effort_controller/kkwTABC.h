#include "rclcpp/rclcpp.hpp"
class kkwTABC
{
 private:
 public:
	static double dzn(const double&A, const double& x) 
	{
		if     ( x>  A ) return x - A;
		else if( x>=-A ) return 0;
		else            return x + A;
	}
	static double sat(const double&A, const double& x) 
	{
		if     ( x>  A ) return A;
		else if( x>=-A ) return x;
		else             return -A;
	}
	static double mclip(const double& a, const double& x) // proj([0,a],x)
	{
		if(a>=x && x>=0) return x;
		if(0>=a && a>=x) return a;
		if(x>=0 && 0>=a) return 0;
		if(0>=x && x>=a) return x;
		if(x>=a && a>=0) return a;
		if(a>=0 && 0>=x) return 0;
		return 0;
	}
private:
public:
	int    first_use ;
	double qx1 ; // proxy position, one step ago
	double ux1 ; // proxy velocity, one step ago
	double qd1 ; // reference position, one step ago
	double qd2 ; // reference position, two steps ago
	double qs1 ; // actual position, one step ago
	double a1  ; // error integral, one step ago

public:
	double T, K, B, L, F, M; 
	double MV, BV, KV, FV;
	int    secB, secC;
	double work(const double& fd, const double& qd, const double& fs, const double& qs, double* p_qx=0, double* p_fa=0)
	{
		double Kh   = K+B/T+L*T;
		double MTT  = M/T/T    ;
		if(first_use){qs1=qs;qx1=qs;ux1=a1=0;first_use=0;}
		/**********************************************/
		double uxa = (MV*ux1+ T*KV*(qd-qx1) + T*(fd+fs) +BV*(qd-qd1) +MV*(qd-2*qd1+qd2)/T    )/(MV+BV*T+KV*T*T);
		uxa = dzn(T*FV/(MV+BV*T+KV*T*T),uxa) ;
		double qxa = qx1 + uxa*T;
		double phi = (B/T)*(qx1-qs1) - L*a1 ;
		double fa,f,qx;
		if(secC)
		{
			double faff = sat(F,M*(uxa-ux1)/T) ;
			double qsa = qs+(phi-faff)/Kh;
			fa  = Kh*(qxa-qsa);
			f   = sat(F,fa);
			qx  = qsa + f/Kh;
		}
		else
		{
			double faff = M*(qs-qx1-T*ux1)/(T*T) ;
			double qsa = qs+(phi-faff)/(Kh+MTT);
			fa  = (Kh+MTT)*(qxa-qsa);
			f   = sat(F,fa);
			qx  = qsa + f/(Kh+MTT);
		}
		double ux  = (qx-qx1)/T   ;
		if(secB) ux = mclip(uxa,ux);
		double a   = a1 + T*(qx-qs)  ;
		/**********************************************/
		qs1=qs; qx1=qx; ux1=ux; a1=a; qd2=qd1; qd1=qd; 
		if(p_qx !=0)*p_qx=qx;
		if(p_fa !=0)*p_fa=fa;
		return f;
	};

public:
	void reset(double qs=0)	
	{
		qs1=qs;qx1=qs;a1=0;ux1=0;
		first_use  = 1;
		secB =1;
		secC =1;
	}
	kkwTABC(){reset();}
};
/**************************************************************/

