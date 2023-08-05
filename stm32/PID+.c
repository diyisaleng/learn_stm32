//此处需要插入上面的单级PID相关代码
 typedef struct
{
   	float kp,ki,kd;//三个系数
    float error,lastError;//误差、上次误差
    float integral,maxIntegral;//积分、积分限幅
    float output,maxOutput;//输出、输出限幅
}PID;

void PID_Init(PID *pid,float p,float i,float d,float maxI,float maxOut)
{
    pid->kp=p;
    pid->ki=i;
    pid->kd=d;
    pid->maxIntegral=maxI;
    pid->maxOutput=maxOut;
}

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
//串级PID的结构体，包含两个单级PID
typedef struct
{
    PID inner;//内环
    PID outer;//外环
    float output;//串级输出，等于inner.output
}CascadePID;
 
//串级PID的计算函数
//参数(PID结构体,外环目标值,外环反馈值,内环反馈值)
void PID_CascadeCalc(CascadePID *pid,float outerRef,float outerFdb,float innerFdb)
{
    PID_Calc(&pid->outer,outerRef,outerFdb);//计算外环
    PID_Calc(&pid->inner,pid->outer.output,innerFdb);//计算内环
    pid->output=pid->inner.output;//内环输出就是串级PID的输出
}
 
CascadePID mypid;//创建串级PID结构体变量
 
int main()
{
    //...其他初始化代码
    PID_Init(&mypid.inner,10,0,0,0,1000);//初始化内环参数
    PID_Init(&mypid.outer,5,0,5,0,100);//初始化外环参数
    while(1)//进入循环运行
    {
        float outerTarget=...;//获取外环目标值
        float outerFeedback=...;//获取外环反馈值
        float innerFeedback=...;//获取内环反馈值
        PID_CascadeCalc(&mypid,outerTarget,outerFeedback,innerFeedback);//进行PID计算
        设定执行机构输出大小(mypid.output);
        delay(10);//延时一段时间
    }
}