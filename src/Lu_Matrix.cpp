//#include "stdafx.h"
#include "Lu_Matrix.h"

Lu_Matrix::Lu_Matrix(long r, long c){
	rowN=r;
	colN=c;
	if(rowN*colN>0)	data=new double[rowN*colN];
	Reset();
}

Lu_Matrix::Lu_Matrix(const Lu_Matrix& m){
	rowN=m.rowN;
	colN=m.colN;
	if(rowN*colN>0) data=new double[rowN*colN];
	for(long i=0;i<rowN*colN;i++){
		data[i]=m.data[i];
	}
}

Lu_Matrix& Lu_Matrix::operator =(const Lu_Matrix& m){
	if(this!=&m){
		if((rowN!=m.rowN)||(colN!=m.colN))
		{
			if(rowN*colN>0) delete[] data;
			rowN=m.rowN;
			colN=m.colN;
			if(rowN*colN>0) data=new double[rowN*colN];
		}
		for(long i=0;i<rowN*colN;i++){
			data[i]=m.data[i];
		}		
	}
	return *this;
}

int Lu_Matrix::Reset(){
	if(rowN*colN>0){
		for(long i=0;i<rowN*colN;i++){
			data[i]=0;
		}
	}
	return 0;
}

int Lu_Matrix::Set(long r, long c){
	if(rowN*colN>0)
	delete []data;
	rowN=r;
	colN=c;
	if(rowN*colN>0)	data=new double[rowN*colN];

	return Reset();		// The return of Reset() is transmitted to the return of Set()
}
int Lu_Matrix::Set(long r)
{
	if(rowN*colN>0)
	delete []data;
	rowN=r;
	colN=1;
	if(rowN*colN>0)	data=new double[rowN*colN];
	return Reset();		// The return of Reset() is transmitted to the return of Set()
}
Lu_Matrix Lu_Matrix::Size() const{
	Lu_Matrix M2by1(2,1);
	M2by1(0)=this->rowN;
	M2by1(1)=this->colN;
	return M2by1;
}
int Lu_Matrix::Length() const{return ((this->colN) * (this->rowN));}
int Lu_Matrix::RowNum() const{return (this->rowN);}
int Lu_Matrix::ColNum() const{return (this->colN);}
double Lu_Matrix::MaxValue() const{
	double MaxVal=this->data[0];
	for(long i=0;i<rowN;i++){
	for(long j=0;j<colN;j++){
	if(MaxVal<data[i*colN+j]){MaxVal=data[i*colN+j];}
	}}	
	return MaxVal;
}
double Lu_Matrix::MaxValue(long int & MaxIndex_row,long int & MaxIndex_col) const{
	MaxIndex_col=0;
	MaxIndex_row=0;
	double MaxVal=this->data[0];
	for(long i=0;i<rowN;i++){
	for(long j=0;j<colN;j++){
	if(MaxVal<data[i*colN+j]){MaxVal=data[i*colN+j];MaxIndex_row=i;MaxIndex_col=j;
	}}}	
	return MaxVal;
}
double Lu_Matrix::MinValue() const{
	double MinVal=this->data[0];
	for(long i=0;i<rowN;i++){
	for(long j=0;j<colN;j++){
	if(MinVal>data[i*colN+j]){MinVal=data[i*colN+j];}
	}}	
	return MinVal;
}
double Lu_Matrix::MinValue(long int & MinIndex_row,long int & MinIndex_col) const{
	MinIndex_col=0;
	MinIndex_row=0;
	double MinVal=this->data[0];
	for(long i=0;i<rowN;i++){
	for(long j=0;j<colN;j++){
	if(MinVal>data[i*colN+j]){MinVal=data[i*colN+j];MinIndex_row=i;MinIndex_col=j;
	}}}	
	return MinVal;
}
Lu_Matrix Lu_Matrix::Abs() const{
	Lu_Matrix abs_m(*this);
	for(long i=0;i<abs_m.RowNum();i++){
		for(long j=0;j<abs_m.ColNum();j++){
			abs_m(i,j)=abs(abs_m(i,j));
		}
	}
	return abs_m;
}

bool Lu_Matrix::SetArrayRow(double a[], int n){
	int m=(this->colN)*(this->rowN);
	for(int i=0;i<m;i++){
		if(i<n)this->data[i]=a[i];
		else this->data[i]=0;
	}
	return true;
}

double & Lu_Matrix::operator()(long i,long j){
	if(rowN*colN<=0)
	{
		rowN=i+1;colN=j+1;
		data=new double[rowN*colN];
		Reset();
	} 
	if(i>rowN-1)i=rowN-1;
	if(j>colN-1)j=colN-1;
	if(i<0)i=0;if(j<0)j=0;

	return *(data+i*(this->colN)+j);
}
double Lu_Matrix::operator()(long i,long j) const{
	if(rowN*colN<=0)
	{
		return 0;
	} 
	if(i>rowN-1)i=rowN-1;
	if(j>colN-1)j=colN-1;
	if(i<0)i=0;if(j<0)j=0;

	return *(data+i*(this->colN)+j);
}


Lu_Matrix& Lu_Matrix::operator +=(const Lu_Matrix & m){
	if(rowN==m.rowN && colN==m.colN){
		for(long i=0;i<rowN;i++){
			for(long j=0;j<colN;j++){
				data[i*colN+j]+=m.data[i*colN+j];
			}
		}
	}else{
		//this->Reset();
	}
	return *this;
}

Lu_Matrix& Lu_Matrix::operator -=(const Lu_Matrix & m){
	if(rowN==m.rowN && colN==m.colN){
		for(long i=0;i<rowN;i++){
			for(long j=0;j<colN;j++){
				data[i*colN+j]-=m.data[i*colN+j];
			}
		}
	}else{
		//this->Reset();
	}
	return *this;
}

Lu_Matrix& Lu_Matrix::operator *=(const Lu_Matrix & m){
	Lu_Matrix rm;
	rm.Set(rowN, m.colN);
	long i,j,k;
	if(colN==m.rowN){
		for(i=0;i<rowN;i++){
			for(j=0;j<m.colN;j++){
				//rm.data[i*m.colN+j]=0;
				for(k=0;k<colN;k++){
					rm.data[i*m.colN+j]+=data[i*colN+k]*m.data[k*m.colN+j];
				}
			}
		}
	}
	return *this=rm;
}

Lu_Matrix& Lu_Matrix::operator /=(Lu_Matrix m1){
	Lu_Matrix  CopyThis(*this);
	long	r, c, itrN;
	double	mElemt, tmp;
	long	mIndex;
	
	if(m1.rowN!=rowN || m1.colN>m1.rowN){ return *this=CopyThis; }

	if(m1.colN<m1.rowN){
		*this	=Tp(m1)*(*this);
		m1		=Tp(m1)*m1;
	}

	for(itrN=0;itrN<rowN;itrN++){
		mElemt=fabs(m1.data[itrN*m1.colN+itrN]);
		mIndex=itrN;
		for(r=itrN+1;r<rowN;r++){
			if(mElemt<fabs(m1.data[r*m1.colN+itrN])){
				mElemt=fabs(m1.data[r*m1.colN+itrN]);
				mIndex=r; 
			}
		}		
		if(mElemt<1.0e-15){ return *this=CopyThis;	}//Make sure that the major element in the column is nonzero
		
		if(mIndex!=itrN){
			for(c=0;c<m1.colN;c++){
				tmp=m1.data[itrN*m1.colN+c];
				m1.data[itrN*m1.colN+c]=m1.data[mIndex*m1.colN+c];
				m1.data[mIndex*m1.colN+c]=tmp;				
			}
			for(c=0;c<colN;c++){
				tmp=data[itrN*colN+c];
				data[itrN*colN+c]=data[mIndex*colN+c];
				data[mIndex*colN+c]=tmp;
			}
		}
		for(r=0;r<rowN;r++){
			tmp	=m1.data[r*m1.colN+itrN]/m1.data[itrN*m1.colN+itrN];
			for(c=0;c<m1.colN;c++){
				if(r!=itrN){
					m1.data[r*m1.colN+c]-=tmp*m1.data[itrN*m1.colN+c];
				}
			}
			for(c=0;c<colN;c++){
				if(r!=itrN){
					data[r*colN+c]-=tmp*data[itrN*colN+c];
				}
			}
		}
		mElemt	=m1.data[itrN*m1.colN+itrN];
		for(c=0;c<m1.colN;c++){
			m1.data[itrN*m1.colN+c]/=mElemt;				
		}
		for(c=0;c<colN;c++){
			data[itrN*colN+c]/=mElemt;			
		}
	}	

	return *this;
}

Lu_Matrix& Lu_Matrix::operator +=(double d){
	for(long i=0;i<rowN;i++){
		for(long j=0;j<colN;j++){
			data[i*colN+j]+=d;
		}
	}
	return *this;
}

Lu_Matrix& Lu_Matrix::operator -=(double d){
	for(long i=0;i<rowN;i++){
		for(long j=0;j<colN;j++){
			data[i*colN+j]-=d;
		}
	}
	return *this;
}

Lu_Matrix& Lu_Matrix::operator *=(double d){
	for(long i=0;i<rowN;i++){
		for(long j=0;j<colN;j++){
			data[i*colN+j]*=d;
		}
	}
	return *this;
}

Lu_Matrix& Lu_Matrix::operator /=(double d){
	for(long i=0;i<rowN;i++){
		for(long j=0;j<colN;j++){
			data[i*colN+j]/=d;
		}
	}
	return *this;
}

int Lu_Matrix::Display() const{
	long i,j;
	
	for(i=0;i<rowN;i++){
		for(j=0;j<colN;j++){
//			cout<<data[i*colN+j]<<",\t";
		}
//		cout<<endl;
	}
//	cout<<endl;
	return 0;
}

int Lu_Matrix::Ut(){
	if(rowN==colN){
		for(long i=0;i<rowN;i++){
			for(long j=0;j<colN;j++){
				if(i==j){
					data[i*colN+j]=1.0;
				}else{
					data[i*colN+j]=0.0;
				}
			}
		}
		return 0;
	}else{
		return 1;
	}
}

int Lu_Matrix::Inv(){
	Lu_Matrix	CopyThis(*this);
	long	r, c, itrN;
	double	mElemt, tmp;
	long	mIndex;
	
	//Lu_Matrix rm=*this;
	Lu_Matrix m1=*this;
	if(rowN!=colN){
		*this=CopyThis;
		return 1;		//If not a square Lu_Matrix, return the Lu_Matrix itself without solving its inverse
	}

	this->Ut();
	//rm.Ut();
	for(itrN=0;itrN<rowN;itrN++){
		mElemt=fabs(m1.data[itrN*colN+itrN]);
		mIndex=itrN;
		for(r=itrN+1;r<rowN;r++){
			if(mElemt<fabs(m1.data[r*colN+itrN])){
				mElemt=fabs(m1.data[r*colN+itrN]);
				mIndex=r; 
			}
		}
		if(mElemt<1.0e-15){
			*this=CopyThis;
			return 2;	//Make sure that the major element in the column is nonzero
		}
		
		if(mIndex!=itrN){
			for(c=0;c<colN;c++){
				tmp=m1.data[itrN*colN+c];
				m1.data[itrN*colN+c]=m1.data[mIndex*colN+c];
				m1.data[mIndex*colN+c]=tmp;			
				tmp=data[itrN*colN+c];
				data[itrN*colN+c]=data[mIndex*colN+c];
				data[mIndex*colN+c]=tmp;
			}
		}
		for(r=0;r<rowN;r++){
			tmp	=m1.data[r*colN+itrN]/m1.data[itrN*colN+itrN];
			for(c=0;c<colN;c++){
				if(r!=itrN){
					m1.data[r*colN+c]-=tmp*m1.data[itrN*colN+c];
					data[r*colN+c]-=tmp*data[itrN*colN+c];
				}				
			}
		}
		mElemt	=m1.data[itrN*colN+itrN];
		for(c=0;c<colN;c++){
			m1.data[itrN*colN+c]/=mElemt;
			data[itrN*colN+c]/=mElemt;				
		}
	}

	return 0;
}

Lu_Matrix Lu_Matrix::GetRowVec(long r) const{
	Lu_Matrix	rm(1, colN);
	long	rIndex=r;
	if(rIndex<0){
		rIndex=0;
	}else if(rIndex>rowN-1){
		rIndex=rowN-1;
	}
	for(long j=0;j<colN;j++){
		rm.data[j]=data[rIndex*colN+j];
	}
	return rm;
}
Lu_Matrix Lu_Matrix::GetColVec(long c) const{
	Lu_Matrix rm(rowN, 1);
	long cIndex=c;
	if(cIndex<0){
		cIndex=0;
	}else if(cIndex>colN-1){
		cIndex=colN-1;
	}
	for(long i=0;i<rowN;i++){
		rm.data[i]=data[i*colN+cIndex];
	}
	return rm;
}

////////////////////////////////////////////////////////////
//
//
////////////////////////////////////////////////////////////
Lu_Matrix operator -(const Lu_Matrix & m){
	Lu_Matrix rm(m);
	for(long i=0;i<rm.rowN;i++){
		for(long j=0;j<rm.colN;j++){
			rm.data[i*rm.colN+j]=-m.data[i*m.colN+j];
		}
	}
	return rm;
}

Lu_Matrix operator+(const Lu_Matrix & m1,const Lu_Matrix & m2){
	Lu_Matrix rm=m1;
	return rm+=m2;
}
Lu_Matrix operator+(const Lu_Matrix & m, double d){
	Lu_Matrix rm=m;
	return rm+=d;
}
Lu_Matrix operator+(double d,const Lu_Matrix & m){
	Lu_Matrix rm=m;
	return rm+=d;
}

Lu_Matrix operator-(const Lu_Matrix & m1,const Lu_Matrix & m2){
	Lu_Matrix rm=m1;
	return rm-=m2;
}
Lu_Matrix operator-(const Lu_Matrix & m, double d){
	Lu_Matrix rm=m;
	return rm-=d;
}
Lu_Matrix operator-(double d,const Lu_Matrix & m){
	Lu_Matrix rm=m;
	rm-=d;
	return -rm;
}

Lu_Matrix operator*(const Lu_Matrix & m1, const Lu_Matrix & m2){
	Lu_Matrix rm=m1;
	return rm*=m2;
}
Lu_Matrix operator*(const Lu_Matrix & m, double d){
	Lu_Matrix rm=m;
	return rm*=d;
}
Lu_Matrix operator*(double d,const Lu_Matrix & m){
	Lu_Matrix rm=m;
	return rm*=d;
}

Lu_Matrix operator/(Lu_Matrix m, Lu_Matrix m1){
	Lu_Matrix rm=m;
	return rm/=m1;
}
Lu_Matrix operator/(Lu_Matrix m, double d){
	Lu_Matrix rm=m;
	return rm/=d;
}

Lu_Matrix Tp(const Lu_Matrix & m){
	Lu_Matrix rm(m.colN, m.rowN);
	for(long i=0;i<rm.rowN;i++){
		for(long j=0;j<rm.colN;j++){
			rm.data[i*rm.colN+j]=m.data[j*m.colN+i];
		}
	}
	return rm;
}

Lu_Matrix Inv(const Lu_Matrix & m){
	long	r, c, itrN;
	double	mElemt, tmp;
	long	mIndex;
	long	rowN=m.rowN;
	long	colN=m.colN;

	Lu_Matrix CopyM=m;
	Lu_Matrix rm=m;
	Lu_Matrix m1=m;
	if(rowN!=colN) return CopyM;		//If not a square Lu_Matrix, return the Lu_Matrix itself without solving its inverse

	rm.Ut();
	for(itrN=0;itrN<rowN;itrN++){
		mElemt=fabs(m1.data[itrN*colN+itrN]);
		mIndex=itrN;
		for(r=itrN+1;r<rowN;r++){
			if(mElemt<fabs(m1.data[r*colN+itrN])){
				mElemt=fabs(m1.data[r*colN+itrN]);
				mIndex=r; 
			}
		}
		if(mElemt<1.0e-15) return CopyM;	//Make sure that the major element in the column is nonzero
		
		if(mIndex!=itrN){
			for(c=0;c<colN;c++){
				tmp=m1.data[itrN*colN+c];
				m1.data[itrN*colN+c]=m1.data[mIndex*colN+c];
				m1.data[mIndex*colN+c]=tmp;			
				tmp=rm.data[itrN*colN+c];
				rm.data[itrN*colN+c]=rm.data[mIndex*colN+c];
				rm.data[mIndex*colN+c]=tmp;
			}
		}
		for(r=0;r<rowN;r++){
			tmp	=m1.data[r*colN+itrN]/m1.data[itrN*colN+itrN];
			for(c=0;c<colN;c++){
				if(r!=itrN){
					m1.data[r*colN+c]-=tmp*m1.data[itrN*colN+c];
					rm.data[r*colN+c]-=tmp*rm.data[itrN*colN+c];
				}				
			}
		}
		mElemt	=m1.data[itrN*colN+itrN];
		for(c=0;c<colN;c++){
			m1.data[itrN*colN+c]/=mElemt;
			rm.data[itrN*colN+c]/=mElemt;				
		}
	}

	return rm;
}
Lu_Matrix Inv2by2Matrix(const Lu_Matrix & m)
{
	if(m.rowN==2&&m.colN==2){
		Lu_Matrix adj(2,2);
		adj(0,0)=m(1,1);adj(1,1)=m(0,0);
		adj(0,1)=-m(0,1);adj(1,0)=-m(1,0);
		double det=Det(m);
		if(det!=0)
		return adj/det;
		else
			return adj*1000000000000;
	}
	return m;
}
Lu_Matrix Inv3by3Matrix(const Lu_Matrix & m)
{
	if(m.rowN==3&&m.colN==3){
		Lu_Matrix adj(3,3);
		adj(0,0)=m(1,1)*m(2,2)-m(1,2)*m(2,1);adj(0,1)=-(m(0,1)*m(2,2)-m(0,2)*m(2,1));  adj(0,2)=(m(0,1)*m(1,2)-m(1,1)*m(0,2));
		adj(1,0)=-(m(1,0)*m(2,2)-m(2,0)*m(1,2));adj(1,1)=(m(0,0)*m(2,2)-m(2,0)*m(0,2));adj(1,2)=-(m(0,0)*m(1,2)-m(1,0)*m(0,2));
		adj(2,0)=m(1,0)*m(2,1)-m(2,0)*m(1,1);adj(2,1)=-(m(0,0)*m(2,1)-m(2,0)*m(0,1));  adj(2,2)=(m(0,0)*m(1,1)-m(1,0)*m(0,1));

		
		double det=Det(m);
		if(fabs(det)>1e-100)
		return adj/det;
		else
			return adj*1e100;
	}
		return m;
	
}
double Det(const Lu_Matrix & m){
	long	r, c, itrN;
	double	mElemt, tmp;
	long	mIndex;
	long	rowN=m.rowN;
	long	colN=m.colN;

	double rd;
	Lu_Matrix m1=m;
	if(rowN!=colN) return 0;		//If not a square Lu_Matrix, return the Lu_Matrix itself without solving its inverse

	for(itrN=0;itrN<rowN;itrN++){
		mElemt=fabs(m1.data[itrN*colN+itrN]);
		mIndex=itrN;
		for(r=itrN+1;r<rowN;r++){
			if(mElemt<fabs(m1.data[r*colN+itrN])){
				mElemt=fabs(m1.data[r*colN+itrN]);
				mIndex=r; 
			}
		}
		if(mElemt<1.0e-15) return 0;	//Make sure that the major element in the column is nonzero
		
		if(mIndex!=itrN){
			for(c=0;c<colN;c++){
				tmp=m1.data[itrN*colN+c];
				m1.data[itrN*colN+c]=m1.data[mIndex*colN+c];
				m1.data[mIndex*colN+c]=tmp;	
			}
		}
		for(r=itrN+1;r<rowN;r++){
			tmp	=m1.data[r*colN+itrN]/m1.data[itrN*colN+itrN];
			for(c=0;c<colN;c++){
				m1.data[r*colN+c]-=tmp*m1.data[itrN*colN+c];		
			}
		}
	}
	rd=1.0;
	for(itrN=0;itrN<rowN;itrN++){
		rd*=m1.data[itrN*colN+itrN];
	}

	return rd;
}

Lu_Matrix GetRowVec(const Lu_Matrix & m, long r){
	return m.GetRowVec(r);
}
Lu_Matrix GetColVec(const Lu_Matrix & m, long c){
	return m.GetColVec(c);
}

Lu_Matrix AbsOfMatrix(const Lu_Matrix & m){
	Lu_Matrix abs_m=m;
	for(long i=0;i<abs_m.ColNum();i++){
		for(long j=0;j<abs_m.RowNum();j++){
			abs_m(i,j)=abs(abs_m(i,j));
		}
	}
	return abs_m;
}
double SumOfMatrix(const Lu_Matrix & m){
	double sum_m=0;
	for(long i=0;i<(m.ColNum());i++){
		for(long j=0;j<m.RowNum();j++){
			sum_m+=(m(i,j));
		}
	}
	return sum_m;
}

//\CF\C2\C3\E6\CA\C7\C7\F3\BD\E2\BE\D8\D5\F3\CC\D8\D5\F7ֵ\BA\CD\CC\D8\D5\F7\CF\F2\C1\BF\B5\C4ʵ\CF֣\BA
//[cpp] view plaincopyprint?
/** 
* @brief \C7\F3ʵ\B6Գƾ\D8\D5\F3\B5\C4\CC\D8\D5\F7ֵ\BC\B0\CC\D8\D5\F7\CF\F2\C1\BF\B5\C4\D1Ÿ\F1\B1ȷ\A8  
03.* \C0\FB\D3\C3\D1Ÿ\F1\B1\C8(Jacobi)\B7\BD\B7\A8\C7\F3ʵ\B6Գƾ\D8\D5\F3\B5\C4ȫ\B2\BF\CC\D8\D5\F7ֵ\BC\B0\CC\D8\D5\F7\CF\F2\C1\BF  
04.* @param a      \B3\A4\B6\C8Ϊn*n\B5\C4\CA\FD\D7飬\B4\E6\B7\C5ʵ\B6Գƾ\D8\D5󣬷\B5\BB\D8ʱ\B6Խ\C7\CFߴ\E6\B7\C5n\B8\F6\CC\D8\D5\F7ֵ  
05.* @param n      \BE\D8\D5\F3\B5Ľ\D7\CA\FD  
06.* @param v      \B3\A4\B6\C8Ϊn*n\B5\C4\CA\FD\D7飬\B7\B5\BB\D8\CC\D8\D5\F7\CF\F2\C1\BF(\B0\B4\C1д洢)  
07.* @param eps    \BF\D8\D6ƾ\AB\B6\C8Ҫ\C7\F3  
08.* @param jt     \D5\FB\D0ͱ\E4\C1\BF\A3\AC\BF\D8\D6\C6\D7\EE\B4\F3\B5\FC\B4\FA\B4\CE\CA\FD  
09.* @return \B7\B5\BB\D8false\B1\EDʾ\B3\AC\B9\FD\B5\FC\B4\FAjt\B4\CE\C8\D4δ\B4ﵽ\BE\AB\B6\C8Ҫ\C7󣬷\B5\BB\D8true\B1\EDʾ\D5\FD\B3\A3\B7\B5\BB\D8  
10.*/  
int Eejcb(double a[], int n, double v[], double eps, int jt)  
{   
    int i,j,p,q,u,w,t,s;   
    double fm,cn,sn,omega,x,y,d;   
  
   int LoopCnt=1;   
    //\B3\F5ʼ\BB\AF\CC\D8\D5\F7\CF\F2\C1\BF\BE\D8\D5\F3Ϊ\B5\A5λ\D5\F3  
    for(i=0; i<=n-1; i++)   
    {     
        v[i*n+i] = 1.0;   
        for(j=0; j<=n-1; j++)   
        {   
            if(i != j)     
                v[i*n+j]=0.0;   
        }   
    }   
  
    while(true)   //ѭ\BB\B7   
    {     
        fm = 0.0;   
        for(i=0; i<=n-1; i++)   //\D5ҳ\F6,\BE\D8\D5\F3a(\CC\D8\D5\F7ֵ),\D6г\FD\B6Խ\C7\CF\DF\CD\E2\C6\E4\CB\FBԪ\CBص\C4\D7\EE\B4\F3\BE\F8\B6\D4ֵ   
        {   
            //\D5\E2\B8\F6\D7\EE\B4\F3ֵ\CA\C7λ\D3\DAa[p][q] ,\B5\C8\D3\DAfm   
            for(j=0; j<=n-1; j++)   
            {   
                d = fabs(a[i*n+j]);   
                if((i!=j) && (d> fm))   
                {   
                    fm = d;     
                    p = i;     
                    q = j;   
                }   
            }   
        }   
  
        if(fm < eps)     //\BE\AB\B6ȸ\B4\BA\CFҪ\C7\F3   
            return LoopCnt; //\D5\FD\B3\A3\B7\B5\BB\D8   
  
        if(LoopCnt > jt)       //\B5\FC\B4\FA\B4\CE\CA\FD̫\B6\E0   
            return LoopCnt;//ʧ\B0ܷ\B5\BB\D8   
  
        LoopCnt ++;       //   \B5\FC\B4\FA\BC\C6\CA\FD\C6\F7   

        u = p*n + q;   
        w = p*n + p;     
        t = q*n + p;     
        s = q*n + q;   
        x = -a[u];   
        y = (a[s]-a[w])/2.0;        //x y\B5\C4\C7󷨲\BBͬ   
        omega = x/sqrt(x*x+y*y);    //sin2\A6\C8   
  
        //tan2\A6\C8=x/y = -2.0*a[u]/(a[s]-a[w])   
        if(y < 0.0)   
            omega=-omega;   
 
        sn = 1.0 + sqrt(1.0-omega*omega); //1+cos2\A6\C8    
        sn = omega /sqrt(2.0*sn);       //sin\A6\C8   
        cn = sqrt(1.0-sn*sn);           //cos\A6\C8   
  
        fm = a[w];   //   \B1任ǰ\B5\C4a[w]   a[p][p]   
        a[w] = fm*cn*cn + a[s]*sn*sn + a[u]*omega;   
        a[s] = fm*sn*sn + a[s]*cn*cn - a[u]*omega;   
        a[u] = 0.0;   
        a[t] = 0.0;   
  
        //   \D2\D4\CF\C2\CA\C7\D0\FDת\BE\D8\D5\F3,\D0\FDת\C1\CB\C1\CBp\D0\D0,q\D0\D0,p\C1\D0,q\C1\D0   
        //   \B5\AB\CA\C7\CBĸ\F6\CC\D8\CA\E2\B5\E3û\D3\D0\D0\FDת(\D5\E2\CBĸ\F6\B5\E3\D4\DA\C9\CF\CA\F6\D3\EF\BE\E4\D6з\A2\C9\FA\C1˱仯)   
        //   \C6\E4\CB\FB\B2\BB\D4\DA\D5\E2Щ\D0к\CD\C1еĵ\E3Ҳû\B1\E4   
        //   \D0\FDת\BE\D8\D5\F3,\D0\FDתp\D0к\CDq\D0\D0   
        for(j=0; j<=n-1; j++)   
        {   
            if((j!=p) && (j!=q))   
            {   
                u = p*n + j;   
                w = q*n + j;   
                fm = a[u];   
                a[u] = a[w]*sn + fm*cn;   
                a[w] = a[w]*cn - fm*sn;   
            }   
        }   
  
        //\D0\FDת\BE\D8\D5\F3,\D0\FDתp\C1к\CDq\C1\D0   
        for(i=0; i<=n-1; i++)   
        {   
            if((i!=p) && (i!=q))   
            {   
                u = i*n + p;     
                w = i*n + q;   
                fm = a[u];   
                a[u]= a[w]*sn + fm*cn;   
                a[w]= a[w]*cn - fm*sn;   
            }   
        }   
  
        //\BC\C7¼\D0\FDת\BE\D8\D5\F3\CC\D8\D5\F7\CF\F2\C1\BF   
        for(i=0; i<=n-1; i++)   
       {   
            u = i*n + p;     
           w = i*n + q;   
           fm = v[u];   
            v[u] =v[w]*sn + fm*cn;   
            v[w] =v[w]*cn - fm*sn;   
       }   
    }   
 
    return LoopCnt;   
}  

int MyEigen(const Lu_Matrix & MMatrix, Lu_Matrix &MyEigenValues, 
					   Lu_Matrix &MyEigenVectors,double deps)  
{  
	int nDem = MMatrix.rowN; 
	if(nDem<1)return 0;
    if(MMatrix.rowN!=MMatrix.colN)
		return 0;
	if((MyEigenValues.colN!=nDem)||(MyEigenValues.colN!=nDem))
		MyEigenValues.Set(nDem,nDem);
	if((MyEigenVectors.colN!=nDem)||(MyEigenVectors.colN!=nDem))
		MyEigenVectors.Set(nDem,nDem);

	
    double *mat = new double[nDem*nDem];    //\CA\E4\C8\EB\BE\D8\D5󣬼\C6\CB\E3\CD\EA\B3\C9֮\BA󱣴\E6\CC\D8\D5\F7ֵ  
    double *eiv = new double[nDem*nDem];    //\BC\C6\CB\E3\CD\EA\B3\C9֮\BA󱣴\E6\CC\D8\D5\F7\CF\F2\C1\BF  
  
    for (int i=0; i<nDem; i++)               //\B8\F8\CA\E4\C8\EB\BE\D8\D5\F3\BA\CD\CA\E4\B3\F6\CC\D8\D5\F7\CF\F2\C1\BF\B5ľ\D8\D5\F3\B8\B3ֵ\B3\F5ʼ\BB\AF  
    {  
        for (int j=0; j<nDem; j++)  
        {  
            mat[i*nDem+j] = MMatrix(i,j);  
            eiv[i*nDem+j] = 0.0;  
        }  
    }  
    int rel = Eejcb(mat, nDem, eiv, deps, 1000);    //\BC\C6\CB\E3\CC\D8\D5\F7ֵ\BA\CD\CC\D8\D5\F7\CF\F2\C1\BF  
	//if (!rel)  
        //return false;
  
    for (int i=0; i<nDem; i++)           //\B8\B3ֵ  
    {  
        for (int j=0; j<nDem; j++)  
        {  
            MyEigenVectors(i,j) = eiv[i*nDem+j];  
  
            if (i == j)  
                MyEigenValues(i,j) = mat[i*nDem+j];  
        }  
    }  
    delete []mat;  
    delete []eiv;  
	return rel;
}


