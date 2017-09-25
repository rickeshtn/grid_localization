#include "ZHU_EKF.h"
ZHU_EKF::ZHU_EKF(){};
ZHU_EKF::ZHU_EKF(int dim_x,Lu_Matrix XX,Lu_Matrix PP){
	init(dim_x,XX,PP);
}


//���UKF״̬�������������������������Ȩϵ�����������ü���ؾ����ά���趨��
void ZHU_EKF::init(int dim_x,Lu_Matrix X0,Lu_Matrix P0){
	dim_X=dim_x;
	Xpre.Set(dim_X,1); 
	P_pre.Set(dim_X,dim_X);
	Xpre=X0;P_pre=P0;
	LastSpeed=0;
	LastTime=0;
	LastYawRate=0;
	FirstUpdateFlag=false;
}
void ZHU_EKF::State_Predict(double CurSpeed,double yawRate,double CurrentTimeInSec,double SpeedError,double yawRateError, double delOdo, double delYaw){
	if(FirstUpdateFlag==false)
	{
		LastSpeed=CurSpeed;
		LastTime=CurrentTimeInSec;
		LastYawRate=yawRate;
	}else{
		
		double DelTime=CurrentTimeInSec-LastTime;
		LastTime=CurrentTimeInSec;//ʱ������
		//double DelMil=0.5*(LastSpeed+CurSpeed)*DelTime;
		double DelMil = delOdo;
		double DelMilCov=DelTime*DelTime*SpeedError*SpeedError;//���ʱ������̼������ķ��
		LastSpeed=CurSpeed;//ʱ������

		//if(DelTime>0.5)DelTime=0;//��ʱ��û�и���ʱ���ߵ���ʱ��Ļ������ϴ󣬲��ã�
		if(DelTime>1)DelTime=0;//��ʱ��û�и���ʱ���ߵ���ʱ��Ļ������ϴ󣬲��ã�
		double DelAngle=0.5*(yawRate+LastYawRate)*DelTime;//���η�����ٶȶ�ʱ��Ļ���
		//double DelAngle = delYaw;
		double DelAngleCov=DelTime*DelTime*yawRateError*yawRateError;//���ʱ����ƫ���������ķ��
		LastYawRate=yawRate;//ʱ������

		//if(DelMil>0.001&&DelMil<10){//�������DelMil̫���̫С����������
		if(true){//�������DelMil̫���̫С����������
			Lu_Matrix U_input(2,1);
			Lu_Matrix Q_input(2,2);
			U_input(0)=DelMil;U_input(1)=DelAngle;//��������
			Q_input(0,0)=DelMilCov;Q_input(1,1)=DelAngleCov;//����Ԫ��Ϊ0����������Э�������
			State_Predict(U_input,Q_input);//״̬���£��������������к�λ����
		}
	}
		
	FirstUpdateFlag=true;
};
/*��������������״̬���̽���״̬����Ԥ��
   ������� U��2*1���󣬵�һ��Ԫ��Ϊ��һ��������̼��������ڶ���Ԫ��Ϊ��һ�����ڳ�yaw�ı仯����������
   ������� Q0: 2*2 matrix,Ϊ����U������Э�������

	*/
void ZHU_EKF::State_Predict(Lu_Matrix U,Lu_Matrix Q_input){
	
	
	Lu_Matrix A(3,3);//���Ի����״̬����
	A(0,0)=1;A(0,1)=0;A(0,2)=-U(0)*sin(Xpre(2)+U(1)/2);
	A(1,0)=0;A(1,1)=1;A(1,2)=U(0)*cos(Xpre(2)+U(1)/2);
	A(2,0)=0;A(2,1)=0;A(2,2)=1;

	Lu_Matrix W(3,2);//���Ի�����������
	W(0,0)=cos(Xpre(2)+U(1)/2);W(0,1)=0;
	W(1,0)=sin(Xpre(2)+U(1)/2);W(1,1)=0;
	W(2,0)=0;                  W(2,1)=1;

	Lu_Matrix buff(3,1);
	buff(0)=U(0)*cos(Xpre(2)+U(1)/2);
	buff(1)=U(0)*sin(Xpre(2)+U(1)/2);
	buff(2)=U(1);
 
	Xpre=Xpre+buff;                     //Xpre����ά�����������浱ǰ״̬����������ĳ���λ�ü���λ
	P_pre=A*P_pre*Tp(A)+W*Q_input*Tp(W);//P_pre: 3*3���󣬱���״̬��Э���
	
	if(Xpre(2)<0) Xpre(2)+=2*PI;//�Ƕȷ�Χ�䵽0-2*PI
	if(Xpre(2)>2*PI) Xpre(2)-=2*PI;
}




//�ò�������ʵ��״̬����,
//����������۲�����������
//������������º��״̬������
void ZHU_EKF::Obv_update(Lu_Matrix Obv,Lu_Matrix R_obv){
	
	Lu_Matrix G(3,3);
	G(0,0)=1;G(0,1)=0;G(0,2)=0;
	G(1,0)=0;G(1,1)=1;G(1,2)=0;
	G(2,0)=0;G(2,1)=0;G(2,2)=1;
	Lu_Matrix V(3,3);
	V(0,0)=1;V(0,1)=0;V(0,2)=0;
	V(1,0)=0;V(1,1)=1;V(1,2)=0;
	V(2,0)=0;V(2,1)=0;V(2,2)=1;

	Lu_Matrix Zpredict=G*Xpre;

	Lu_Matrix K_gain=P_pre*Tp(G)*Inv(G*P_pre*Tp(G)+V*R_obv*Tp(V));

	Obv-=Zpredict;
	if((Obv(2))>PI/6)Obv(2)-=2*PI;
	if((Obv(2))<-PI/6)Obv(2)+=2*PI;

	
	Xpre=Xpre+K_gain*(Obv);
	P_pre=P_pre-K_gain*G*P_pre;
	if(Xpre(2)<0) Xpre(2)+=2*PI;//�Ƕȷ�Χ�䵽0-2*PI
	if(Xpre(2)>2*PI) Xpre(2)-=2*PI;
	
}

bool ZHU_EKF::Obv_GPS_update(double x,double y,Lu_Matrix R0)
{
	Lu_Matrix G(2,3); //�������Zobv=G*Xpre;
	G(0,0)=1;G(0,1)=0;G(0,2)=0;
	G(1,0)=0;G(1,1)=1;G(1,2)=0;

	Lu_Matrix Zpre(2,1);
	Zpre=G*Xpre;//���Ԥ�ⷽ��

	
	Lu_Matrix Zobv(2,1);//�������۲⵽�������
	Zobv(0)=x;
	Zobv(1)=y;

	Zobv-=Zpre;
	if(Zobv(0)*Zobv(0)+Zobv(1)*Zobv(1)<0.00000001)//���Ԥ������ĺ͹۲�Ļ���һ�£���ʲô�����������ء�
	return true;
	
	Lu_Matrix K_gain=P_pre*Tp(G)*Inv(G*P_pre*Tp(G)+R0);//����EKF �������

	Xpre=Xpre+K_gain*(Zobv);//״̬���£�measurement update
	P_pre=P_pre-K_gain*G*P_pre;//״̬��Э����������

	if(Xpre(2)<0) Xpre(2)+=2*PI;//�Ƕȷ�Χ�䵽0-2*PI
	if(Xpre(2)>2*PI) Xpre(2)-=2*PI;

	return true;
}