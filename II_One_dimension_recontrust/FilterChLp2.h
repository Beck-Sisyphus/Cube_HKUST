// http://www.schwietering.com/jayduino/filtuino/
#ifndef __FILTER_CHEBYSHEV_H
#define __FILTER_CHEBYSHEV_H

//Low pass chebyshev filter order=2 alpha1=0.2 
class  FilterChLp2
{
  public:
    FilterChLp2()
    {
      v[0]=0.0;
      v[1]=0.0;
    }
  private:
    float v[3];
  public:
    float step(float x) //class II 
    {
      v[0] = v[1];
      v[1] = v[2];
      v[2] = (2.028501812862580078e-1 * x)
         + (-0.49133099781497924230 * v[0])
         + (0.67993027266994710001 * v[1]);
      return 
         (v[0] + v[2])
        +2 * v[1];
    }
};


//Low pass chebyshev filter order=1 alpha1=1 
class  FilterChLp1_100Hz
{
  public:
    FilterChLp1_100Hz()
    {
      v[0]=0.0;
    }
  private:
    float v[2];
  public:
    float step(float x) //class II 
    {
      v[0] = v[1];
      v[1] = (1.110223024625156540e-16 * x)
         + (1.00000000000000022204 * v[0]);
      return 
         (v[0] + v[1]);
    }
};


#endif
