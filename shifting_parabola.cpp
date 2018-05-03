void shift_parabola()
{
	double a,b,c,x0,y0,x1,y1,m;
	int flag=0;
	//initialize f with 0 if left lane and 1 if right
	for( y0=0; y0<img.rows; y0++)
	{
		x0=a*pow(y0,2)+b*y0+c;
		m=-(2*a*y0+b)
		if(flag)
			m*=-1;
		theta=atan(m);
	    x=x0+(m*cos(theta));
	    y=y0-(m*sin(theta));
	    if(isValid(img,x0,y0))
	    {
	    	img.at<Vec3b>(y,x)[0]=255;
	    	img.at<Vec3b>(y,x)[2]=255;
	    }
  	}
  	return 0;
}