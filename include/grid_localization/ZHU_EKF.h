#ifndef ZHU_EKF_H
#define ZHU_EKF_H

#include "Lu_Matrix.h"
//#include "ADJ_MAG_INFO.h"
/*˵����1��Lu_Matrix �Ǿ����࣬
                �������obj��Ԫ�����÷�ʽΪ��obj(0,0)��ʾ0�У���0��Ԫ�أ�
				                          obj(1,0)��ʾ��1�У���0��Ԫ��
			����Lu_Matrix �����ʱ������ָ������ά�������磺
			        Lu_Matrix obj(3,2)����3*2ά���󣬲�����Ԫ�س�ʼ��Ϊ0��
					�����ָ��ά��������Lu_Matrix obj��������һ��0*0ά�Ŀվ���

        2��EKF�����ǣ�1�����ȶ���ZHU_EKF��Ķ�������ָ���κβ�����
		             2���ٳ�ʼ�������ú�����ZHU_EKF_init��
					 3�����йߵ�����̼�����ʱ����״̬���£����ú�����State_Predict
					 4����gps���ݵ���ʱ���в������£�������Obv_GPS_update
		3����ʼ���������ʱ�̣�����getState������ά״̬��������
		

*/
class ZHU_EKF{
public:
	ZHU_EKF();
	ZHU_EKF(int dim_x,Lu_Matrix X0,Lu_Matrix P0);
	~ZHU_EKF(){};

	/*********�ӿں�����ZHU_EKF_init**************
		�������ܣ����EKF��ʼ����
		���������int dim_x:״̬ά��
				 Lu_Matrix X0����ʼ״̬������
				 Lu_Matrix P0����ʼ״̬Э�������
		�����������
	****************************/
	void init(int dim_x,Lu_Matrix X0,Lu_Matrix P0);
	/*********�ӿں�����State_Predict**************
		���������CurSpeed:��ǰ���� ��λ��m/s
				 yawRate:��ǰ��z����ٶȵ�λ:rad/s, ��ʱ��Ϊ��
				CurrentTimeInSec��ϵͳʱ�䣬��λ �룬��������Ϊms,���Ϊus����
				SpeedError���ٶȴ���������λm/s
				yawRateError:�ߵ�����������λ rad/s
		�����������
	****************************/
	void State_Predict(double CurSpeed,double yawRate,double CurrentTimeInSec,double SpeedError,double yawRateError, double delOdo, double delYaw);

	/**************ֻ�۲� x��y��
		�������˵����double x:GPS���صĺ����꣬��λ ���ס�
	                 double x:GPS���صĺ����꣬��λ ���ס�
	                 Lu_Matrix R0:2*2���󣬶Խ���2��Ԫ�طֱ��Ӧx,y�ķ��ע�⣬�Ǵ��������ȵ�ƽ���������Ǿ��ȣ�����
	***********************/
	bool Obv_GPS_update(double x,double y,Lu_Matrix R0);

	//����״̬
	Lu_Matrix getState(){
		return Xpre;
	}

private:
//Attibutes
	int dim_X;//��������Ϊ��״̬ά�����۲�����ά������������ά�����Լ�sigma��ĸ�����	
	Lu_Matrix Xpre;//���º��״̬��Ԥ���״̬
	Lu_Matrix P_pre;//��һ��״̬��Ӧ��Э�������
	
	double LastSpeed;
	double LastTime;
	double LastYawRate;
	bool   FirstUpdateFlag;

//functions
	
	void State_Predict(Lu_Matrix U_input,Lu_Matrix Q_input);//���������ݽ���״̬����Ԥ��,U:����������Q_input:��������Э���
	
	//ȫ״̬�۲⣬������е�״̬�������ô������۲⣬���ô˺���
	//�������˵����Lu_Matrix Obv��3*1���󣬷ֱ��Ӧ״̬x,y,��
	//             Lu_Matrix R0:3*3���󣬶Խ�������Ԫ�طֱ��Ӧ״̬�ķ��ע�⣬�Ǵ��������ȵ�ƽ���������Ǿ��ȣ�����
	void Obv_update(Lu_Matrix Obv,Lu_Matrix R_obv);   //�ò�������ʵ��״̬����   obv:�۲�����������



};


#endif
