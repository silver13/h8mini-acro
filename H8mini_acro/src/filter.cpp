
#include "config.h"


// These filters were made with the filter calculator at  "http://www.schwietering.com/jayduino/filtuino/"
// the sample rate is 1Khz (loop time)



#ifdef SOFT_LPF_1ST_023HZ
//Low pass bessel filter order=1 alpha1=0.023 
class  FilterBeLp1
{
	public:
		FilterBeLp1()
		{
			v[0]=0.0;
		}
	private:
		float v[2];
	public:
		float step(float x) //class II 
		{
			v[0] = v[1];
			v[1] = (6.749703162983405891e-2f * x)
				 + (0.86500593674033188218f * v[0]);
			return 
				 (v[0] + v[1]);
		}
};

FilterBeLp1 filter[3];

#endif

#ifdef SOFT_LPF_1ST_100HZ
//Low pass bessel filter order=1 alpha1=0.1 
class  FilterBeLp1
{
	public:
		FilterBeLp1()
		{
			v[0]=0.0f;
		}
	private:
		float v[2];
	public:
		float step(float x) //class II 
		{
			v[0] = v[1];
			v[1] = (2.452372752527856026e-1f * x)
				 + (0.50952544949442879485f * v[0]);
			return 
				 (v[0] + v[1]);
		}
};

FilterBeLp1 filter[3];
#endif

#ifdef SOFT_LPF_1ST_043HZ
//Low pass bessel filter order=1 alpha1=0.043 
// 43Hz 1st order
class  FilterBeLp1
{
	public:
		FilterBeLp1()
		{
			v[0]=0.0f;
		}
	private:
		float v[2];
	public:
		float step(float x) //class II 
		{
			v[0] = v[1];
			v[1] = (1.196534197538850347e-1f * x)
				 + (0.76069316049222990284f * v[0]);
			return 
				 (v[0] + v[1]);
		}
};

FilterBeLp1 filter[3];
#endif


#ifdef SOFT_LPF_2ND_043HZ
//Low pass bessel filter order=2 alpha1=0.043
// 43Hz 2nd order
class  FilterBeLp2
{
	public:
		FilterBeLp2()
		{
			v[0]=0.0f;
			v[1]=0.0f;
		}
	private:
		float v[3];
	public:
		float step(float x) //class II 
		{
			v[0] = v[1];
			v[1] = v[2];
			v[2] = (2.248505631563763041e-2f * x)
				 + (-0.54947452981557831642f * v[0])
				 + (1.45953430455302779478f * v[1]);
			return 
				 (v[0] + v[2])
				+2 * v[1];
		}
};

FilterBeLp2 filter[3];
#endif

#ifdef SOFT_LPF_2ND_088HZ
//Low pass bessel filter order=2 alpha1=0.088 
// 88hz 2nd order
class  FilterBeLp2
{
	public:
		FilterBeLp2()
		{
			v[0]=0.0f;
			v[1]=0.0f;
		}
	private:
		float v[3];
	public:
		float step(float x) //class II 
		{
			v[0] = v[1];
			v[1] = v[2];
			v[2] = (7.420263821683734107e-2f * x)
				 + (-0.28777447277621337474f * v[0])
				 + (0.99096391990886401047f * v[1]);
			return 
				 (v[0] + v[2])
				+2 * v[1];
		}
};

FilterBeLp2 filter[3];

#endif

#ifdef SOFT_LPF_4TH_088HZ
//Low pass bessel filter order=4 alpha1=0.088 
class  FilterBeLp4
{
	public:
		FilterBeLp4()
		{
			for(int i=0; i <= 4; i++)
				v[i]=0.0f;
		}
	private:
		float v[5];
	public:
		float step(float x) //class II 
		{
			v[0] = v[1];
			v[1] = v[2];
			v[2] = v[3];
			v[3] = v[4];
			v[4] = (9.903177770434866065e-3f * x)
				 + (-0.07227144852752773185f * v[0])
				 + (0.49032309909991900199f * v[1])
				 + (-1.33200820239952477664f * v[2])
				 + (1.75550570750017564947f * v[3]);
			return 
				 (v[0] + v[4])
				+4 * (v[1] + v[3])
				+6 * v[2];
		}
};

FilterBeLp4 filter[3];
#endif


#ifdef SOFT_LPF_4TH_250HZ
//Low pass bessel filter order=4 alpha1=0.25 
class  FilterBeLp4
{
	public:
		FilterBeLp4()
		{
			for(int i=0; i <= 4; i++)
				v[i]=0.0f;
		}
	private:
		float v[5];
	public:
		float step(float x) //class II 
		{
			v[0] = v[1];
			v[1] = v[2];
			v[2] = v[3];
			v[3] = v[4];
			v[4] = (1.634284827934446571e-1f * x)
				 + (-0.01499351594000761234f * v[0])
				 + (-0.13250019942726620759f * v[1])
				 + (-0.54107991113589737342f * v[2])
				 + (-0.92628209819194329278f * v[3]);
			return 
				 (v[0] + v[4])
				+4 * (v[1] + v[3])
				+6 * v[2];
		}
};
FilterBeLp4 filter[3];
#endif

#ifdef SOFT_LPF_4TH_160HZ
//Low pass bessel filter order=4 alpha1=0.16 
class  FilterBeLp4
{
	public:
		FilterBeLp4()
		{
			for(int i=0; i <= 4; i++)
				v[i]=0.0f;
		}
	private:
		float v[5];
	public:
		float step(float x) //class II 
		{
			v[0] = v[1];
			v[1] = v[2];
			v[2] = v[3];
			v[3] = v[4];
			v[4] = (5.353739685984656299e-2f * x)
				 + (-0.00850622068385239495f * v[0])
				 + (0.06367126996659371041f * v[1])
				 + (-0.31152539409023977113f * v[2])
				 + (0.39976199504995363343f * v[3]);
			return 
				 (v[0] + v[4])
				+4 * (v[1] + v[3])
				+6 * v[2];
		}
};

FilterBeLp4 filter[3];
#endif


extern "C" float lpffilter( float in,int num )
{
	#ifdef SOFT_LPF_NONE
	return in;
	#else
	return filter[num].step(in );
	#endif
	
}


// 16Hz hpf filter for throttle compensation
//High pass bessel filter order=1 alpha1=0.016 
class  FilterBeHp1
{
	public:
		FilterBeHp1()
		{
			v[0]=0.0;
		}
	private:
		float v[2];
	public:
		float step(float x) //class II 
		{
			v[0] = v[1];
			v[1] = (9.521017968695103528e-1f * x)
				 + (0.90420359373902081668f * v[0]);
			return 
				 (v[1] - v[0]);
		}
};

FilterBeHp1 throttlehpf1;

extern "C" float throttlehpf( float in )
{
	return throttlehpf1.step(in );
}



