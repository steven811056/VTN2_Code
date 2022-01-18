#ifndef __VIEW_H__
#define __VIEW_H__

class View
{
private:
	/* data */
public:
	View(/* args */);
	~View();
	void Start(void);
	void Stop(void);
	void Status(void);
	void EvacuateTest(int value , int time);
	void OxygenConerntrationTest(int value , int time);
};

View::View(/* args */)
{
}

View::~View()
{
}



#endif