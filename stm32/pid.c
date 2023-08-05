#define P 10
#define I 1
#define D 5
#define MAXI 800
#define MAXOUT 1000

//首先定义PID结构体用于存放一个PID的数据
typedef struct
{
   	float kp,ki,kd;//三个系数
    float error,lastError;//误差、上次误差
    float integral,maxIntegral;//积分、积分限幅
    float output,maxOutput;//输出、输出限幅
}PID;
 
//用于初始化pid参数的函数
void PID_Init(PID *pid,float p,float i,float d,float maxI,float maxOut)
{
    pid->kp=p;
    pid->ki=i;
    pid->kd=d;
    pid->maxIntegral=maxI;
    pid->maxOutput=maxOut;
}
 
//进行一次pid计算
//参数为(pid结构体,目标值,反馈值)，计算结果放在pid结构体的output成员中
void PID_Calc(PID *pid,float reference,float feedback)
{
 	//更新数据
    pid->lastError=pid->error;//将旧error存起来
    pid->error=reference-feedback;//计算新error
    //计算微分
    float dout=(pid->error-pid->lastError)*pid->kd;
    //计算比例
    float pout=pid->error*pid->kp;
    //计算积分
    pid->integral+=pid->error*pid->ki;
    //积分限幅
    if(pid->integral > pid->maxIntegral) pid->integral=pid->maxIntegral;
    else if(pid->integral < -pid->maxIntegral) pid->integral=-pid->maxIntegral;
    //计算输出
    pid->output=pout+dout+pid->integral;
    //输出限幅
    if(pid->output > pid->maxOutput) pid->output=pid->maxOutput;
    else if(pid->output < -pid->maxOutput) pid->output=-pid->maxOutput;
}
 
PID mypid;//创建一个PID结构体变量
 
int main()
{
    //...这里有些其他初始化代码
    PID_Init(&mypid,P,I,D,MAXI,MAXOUT);//初始化PID参数
    while(1)//进入循环运行
    {
        float feedbackValue=...;//这里获取到被控对象的反馈值
        float targetValue=...;//这里获取到目标值
        PID_Calc(&mypid,targetValue,feedbackValue);//进行PID计算，结果在output成员变量中
        设定执行器输出大小(mypid.output);
        delay(10);//等待一定时间再开始下一次循环
    }
}