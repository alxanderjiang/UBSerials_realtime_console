//�ļ�����satpos
/*���ܣ����㶨λ��غ���*/
/*�����㣬������ӳٽ��㡢�����Ӳ���㡢���Ƿ�λ�Ǹ߶ȽǼ��㡢����λ�ü��㡢���㶨λ*/
#include <stdio.h>
#include <math.h>
#include <string.h>
#include"coordinatetrans.h"
#include"binaryread.h"
//#include"class4.h"
#include"matprocess.h"
#include <math.h>
#define OMGE 7.2921151467E-5
#define GM 3.9860047e14
using namespace std;
//#define f 1.0/298.257223563
#define a 6378137.0
#define pi 3.1415926535897932384626433832795
#define Fc -4.442807633e-10//�����ЧӦ��������



/*����㡢�����㡢���Ǹ߶ȽǷ���Ǽ��㺯��*/

//���º�������Ϊ���ļ��ж�ȡ��������
int getion(const char* filename, double* ion) {
    double iona[8], ionb[8];
    FILE* fp = fopen(filename, "r");
    char ch; char fullstr[100]; int flag = 0;
    while (!feof(fp)) {
        fgets(fullstr, 100, fp);
        char stra[] = { "ALPHA" };
        char strb[] = { "BETA" };
        if (strstr(fullstr, stra))
            sscanf(fullstr, "%lfD%lf %lfD%lf %lfD%lf %lfD%lf", &iona[0], &iona[1], &iona[2], &iona[3], &iona[4], &iona[5], &iona[6], &iona[7]);
        if (strstr(fullstr, strb)) {
            sscanf(fullstr, "%lfD%lf %lfD%lf %lfD%lf %lfD%lf", &ionb[0], &ionb[1], &ionb[2], &ionb[3], &ionb[4], &ionb[5], &ionb[6], &ionb[7]);
            flag = 1;
            break;
        }
    }
    if (flag == 0) return 0;
    fclose(fp);
    ion[0] = iona[0] * (pow(10, iona[1]));
    ion[1] = iona[2] * (pow(10, iona[3]));
    ion[2] = iona[4] * (pow(10, iona[5]));
    ion[3] = iona[6] * (pow(10, iona[7]));
    ion[4] = ionb[0] * (pow(10, ionb[1]));
    ion[5] = ionb[2] * (pow(10, ionb[3]));
    ion[6] = ionb[4] * (pow(10, ionb[5]));
    ion[7] = ionb[6] * (pow(10, ionb[7]));
    return 1;
}
//���º���Ϊ�Ӷ�������Ϣ�ж�ȡ������ӳٲ���
void getion(unsigned char* fullodata, double* ion, breport epoch) {
    int start = epoch.start, end = epoch.end;
    ion[0] = bit2double(fullodata + start + 24);
    ion[1] = bit2double(fullodata + start + 24 + 8);
    ion[2] = bit2double(fullodata + start + 24 + 16);
    ion[3] = bit2double(fullodata + start + 24 + 24);
    ion[4] = bit2double(fullodata + start + 24 + 32);
    ion[5] = bit2double(fullodata + start + 24 + 40);
    ion[6] = bit2double(fullodata + start + 24 + 48);
    ion[7] = bit2double(fullodata + start + 24 + 56);
}
void getion(unsigned char* fullodata, double* ion, breport epoch,const char* OEM7) {
    int start = epoch.start, end = epoch.end;
    ion[0] = bit2double(fullodata + start + 28);
    ion[1] = bit2double(fullodata + start + 28 + 8);
    ion[2] = bit2double(fullodata + start + 28 + 16);
    ion[3] = bit2double(fullodata + start + 28 + 24);
    ion[4] = bit2double(fullodata + start + 28 + 32);
    ion[5] = bit2double(fullodata + start + 28 + 40);
    ion[6] = bit2double(fullodata + start + 28 + 48);
    ion[7] = bit2double(fullodata + start + 28 + 56);
}
//���º�������Ϊ�������Ƿ���Ǻ͸߶Ƚ�
void getazel(const double* rs, const double* rr, double* azel) {
    //�Ӳ����������Ǻͽ��ջ�����ͬʱת��Ϊ�������
    //double ion[3]={},pos[3]={};
    double rsblh[3]; xyztoblh(rs[0], rs[1], rs[2], rsblh);
    double rrblh[3]; xyztoblh(rr[0], rr[1], rr[2], rrblh);//�����
    //��ע��xyztoblh��������ֵ�ĸ�ʽ�ǽǶ��ƻ��ǻ����ƣ�������Ϊ�Ƕ���
    double d = sqrt(pow(rs[0] - rr[0], 2) + pow(rs[1] - rr[1], 2) + pow(rs[2] - rr[2], 2));
    double e1, e2, e3, e[3];

    e1 = (rs[0] - rr[0]) / d; e2 = (rs[1] - rr[1]) / d; e3 = (rs[2] - rr[2]) / d;
    e[0] = rs[0] - rr[0]; e[1] = rs[1] - rr[1]; e[2] = rs[2] - rr[2];
    double B = rrblh[0] * pi / 180, L = rrblh[1] * pi / 180;//��վ�Ĵ�����꣬ת��Ϊ�������Լ������Ǻ���

    //��������ϵתվ������ϵ
    double E, N, U, ER[3] = {};
    double H[9] = { -sin(L),cos(L),0,
                -sin(B) * cos(L),-sin(B) * sin(L),cos(B),
                cos(B) * cos(L),cos(B) * sin(L),sin(B) };

    matx(H, 3, 3, e, 3, 1, ER);
    E = ER[0]; N = ER[1]; U = ER[2];

    //���Ǹ߶Ƚ�/����ǵļ���
    double az, el;
    az = atan2(E, N);
    el = asin(U / d);
    azel[0] = az;
    azel[1] = el;
}
//���º�������Ϊ���������ӳ�
double ionmodel(GPSTime t, const double* ion, const double* pos, const double* azel) {
    //�Ӳ�����ȡ����Ҫ����ֵ
    double az = azel[0];
    double el = azel[1];
    double rrblh[3]; xyztoblh(pos[0], pos[1], pos[2], rrblh);
    //����������Ľ�
    double Phi = 0.0137 / ((el / pi) + 0.11) - 0.022;
    //�������㴩�̵��γ��phi1
    double phi1, phiu;
    phiu = rrblh[0] / 180.0;
    phi1 = phiu + Phi * cos(az);
    if (phi1 > 0.416) phi1 = 0.416;
    else if (phi1 < -0.416) phi1 = -0.416;
    //�������㴩�̵㾭��lamda1
    double lamda1, lamdau;
    lamdau = rrblh[1] / 180.0;
    lamda1 = lamdau + Phi * sin(az) / cos(phi1 * pi);
    //�������㴩�̵�ĵش�γ��phim
    double phim;
    phim = phi1 + 0.064 * cos((lamda1 - 1.617) * pi);
    //�������㴩�̵�ĵ���ʱ��localtime
    double localtime;
    localtime = 43200 * lamda1 + t.Second;//���㵱��ʱ����GPS������Ϊ��׼��
    localtime = localtime - floor(localtime / 86400.0) * 86400;//�۳������������õ�һ���ڵĵط�ʱ����
    //���������ӳٵķ���A1
    double A1;
    A1 = ion[0] + phim * (ion[1] + phim * (ion[2] + phim * ion[3]));
    if (A1 < 0) A1 = 0;
    //���������ӳٵ�����P1
    double P1;
    P1 = ion[4] + phim * (ion[5] + phim * (ion[6] + phim * ion[7]));
    if (P1 < 72000) P1 = 72000;
    //���������ӳ���λX1
    double X1;
    X1 = 2 * pi * (localtime - 50400) / P1;
    //������б����F
    double F;
    F = 1.0 + 16.0 * pow((0.53 - el / pi), 3);

    //ģ�Ͳ���������ϣ��������ģ�ͼ��������ӳ�IL1GPS
    double IL1GPS;
    if (fabs(X1) <= 1.57)
        IL1GPS = clight * (5 * (1e-9) + A1 * (1 - 0.5 * X1 * X1 + pow(X1, 4) / 24.0)) * F;
    else
        IL1GPS = 5 * (1e-9) * clight * F;
    double IGS1 = clight * (5 * (1e-9) + A1 * (1 - 0.5 * X1 * X1 + pow(X1, 4) / 24.0)) * F;
    double IGS2 = 5 * (1e-9) * clight * F;
    return IL1GPS;
}
//���º�������Ϊ�������ӳټ���
double tropmodel(const double* pos, const double* azel) {
    //��ֱ������ת��Ϊ�������
    double posblh[3]; xyztoblh(pos[0], pos[1], pos[2], posblh);
    //���ڵ����ϣ��������ӳٹ���
    if (posblh[2] < -100.0 || 1E4 < posblh[2] || azel[1] <= 0) return 0.0;

    double humi = 0.7;
    double h = posblh[2], b = posblh[0] * pi / 180.0;//��Ϊ��ͷ�ļ�������ת����������Ϊ�Ƕ�ֵ�����Լ���ǰ��Ҫ��ԭ
    if (posblh[2] < 0.0) h = 0.0;//����̹߳��㴦��

    double T = 15.0 - 6.5 * 1e-3 * h + 273.16;
    double e = 6.108 * humi * exp((17.15 * T - 4684.0) / (T - 38.45));
    double p = 1013.25 * pow((1 - 2.2557e-5 * h), 5.2568);
    double z = pi / 2.0 - azel[1];
    double trph = 0.0022768 * p / (cos(z) * (1.0 - 0.00266 * cos(2 * b) - 0.00028 * h / 1000.0));
    double trpw = 0.002277 * (1255.0 / T + 0.05) * e / cos(z);
    double trp = trph + trpw;
    return trp;
}

/*���Ǹ߶Ƚǡ�����ǡ�������ӳ١��������ӳټ��㺯������*/

/*����λ�ü��㺯����ʼ*/

//�ַ���ת����
int char2int(char c) {
    if (c >= '0' && c <= '9')
        return c - '0';
    else
        return 0;
}

//��ȡ������������

void readmessagefile(int name, const char* filename, double* data, int* snum) {

    FILE* fp = fopen(filename, "r");
    char ch; char fullstr[100];

    while (1) {
        fgets(fullstr, 100, fp);
        if (strstr(fullstr, "END OF HEADER"))
            break;
    }
    /*
    for(int i=1;i<=8*(name-1);i++){
      fgets(fullstr,100,fp);
    }//�ҵ�Ŀ�������������ļ��е�λ��
    */
    while (1) {
        fgets(fullstr, 100, fp);
        int fhead = 10 * char2int(fullstr[0]) + char2int(fullstr[1]);
        if (fhead == name)
            break;
    }

    int line = 1;
    while (1) {
        double dataa[8] = {}, datab[8] = {};
        //fgets(fullstr,100,fp);
        if (line == 1) {
            char tc;
            sscanf(fullstr, "%d %d %d %d %d %d %lf %lf%c%lf %lf%c%lf %lf%c%lf", &snum[0], &snum[1], &snum[2], &snum[3], &snum[4], &snum[5], &snum[6], &dataa[0], &tc, &datab[0], &dataa[1], &tc, &datab[1], &dataa[2], &tc, &datab[2]);
            data[1] = dataa[0] * pow(10, datab[0]);
            data[2] = dataa[1] * pow(10, datab[1]);
            data[3] = dataa[2] * pow(10, datab[2]);
        }
        else {
            fgets(fullstr, 100, fp);
            char tc;
            sscanf(fullstr, "%lf%c%lf %lf%c%lf %lf%c%lf %lf%c%lf", &dataa[0], &tc, &datab[0], &dataa[1], &tc, &datab[1], &dataa[2], &tc, &datab[2], &dataa[3], &tc, &datab[3]);
            data[0 + 4 * (line - 1)] = dataa[0] * pow(10, datab[0]);
            data[1 + 4 * (line - 1)] = dataa[1] * pow(10, datab[1]);
            data[2 + 4 * (line - 1)] = dataa[2] * pow(10, datab[2]);
            data[3 + 4 * (line - 1)] = dataa[3] * pow(10, datab[3]);
        }
        line++;

        if (line == 9) break;
    }
    data[0] = snum[0];
    fclose(fp);
};

//��������������ת�Ƶ��ṹ����
eph_t getsate(double* data) {
    eph_t s;
    s.crs = data[5];
    s.deln = data[6];
    s.M0 = data[7];
    s.cuc = data[8];
    s.e = data[9];
    s.cus = data[10];
    s.A = data[11];
    s.toes = data[12];
    s.cic = data[13];
    s.OMG0 = data[14];
    s.cis = data[15];
    s.i0 = data[16];
    s.crc = data[17];
    s.omg = data[18];
    s.OMGD = data[19];
    s.idot = data[20];
    s.week = data[22];
    s.svh = data[25];
    return s;
}

struct gtime_t com2unixtime(struct COMMONTIME t0) {
    struct gtime_t t;
    int i = 0;
    //��ʼ����ͨ��ʱ��Unixtime���ʱ��Days
    int Days = 0;
    int doy[] = { 1,32,60,91,121,152,182,213,244,274,305,335 };
    Days = (t0.Year - 1970) * 365 + (t0.Year - 1969) / 4 + doy[t0.Month - 1] + t0.Day - 2 + (t0.Year % 4 == 0 && t0.Month >= 3 ? 1 : 0);//��ʦ�ṩ�㷨����2000����ǰʱ����ֳ���ʱ������
    t.time = Days * 86400 + t0.Hour * 3600 + t0.Minute * 60 + floor(t0.Second);
    t.second = t0.Second - floor(t0.Second);
    return  t;
}
//��RINEX�ļ��ж�ȡ�۲�ʱ���α��
int gettobsandrho(char* filename, char country, int name, gtime_t& t, const double& rho, int rank) {
    //��filename��Ӧ�Ĺ۲��ļ��в���country���ҵ�name�����ǣ�����ȡ��������۲��ļ��еĵ�rank����Ԫ��α��͹۲�ʱ��

    FILE* fp = fopen(filename, "r");
    char fullstr[100];
    char prn[5] = { country };
    if (name <= 9) {
        prn[1] = '0';
        prn[2] = '0' + name;
    }
    else {
        prn[1] = '0' + name / 10;
        prn[2] = '0' + name % 10;
    }

    while (1) {
        fgets(fullstr, 100, fp);
        if (strstr(fullstr, "END OF HEADER"))
            break;
    }
    struct COMMONTIME temp;
    int rankflag = 0;
    while (1) {
        fgets(fullstr, 100, fp);
        char tflag[5] = { ">" };
        if (strstr(fullstr, tflag)) rankflag++;
        if (rankflag == rank) {
            sscanf(fullstr, "> %d %d %d %d %d %lf", &temp.Year, &temp.Month, &temp.Day, &temp.Hour, &temp.Minute, &temp.Second);
            break;
        }
    }
    //sscanf(fullstr,"> %d %d %d %d %d %lf",&temp.Year,&temp.Month,&temp.Day,&temp.Hour,&temp.Minute,&temp.Second);
    t = com2unixtime(temp);
    int findflage = 0;
    while (1) {
        if (fgets(fullstr, 100, fp) == NULL)
            break;
        if (strstr(fullstr, prn)) {
            findflage = 1;
            char s[3];
            sscanf(fullstr, "%s %lf", &s, &rho);
            break;
        }
    }
    fclose(fp);
    if (findflage == 0)
        printf("δ���յ�%s�����ź�", prn);

    return findflage;
}
//ֱ��Ӧ��α������(�ṹ��)��ȡα��
int gettobsandrho(double* R, char country, int name, double& rho) {
    if (R[name] < 200000) { printf("δ���յ�%c%02d�����ź�\n", country, name); return 0; }//�޶�Ӧα��
    else {
        rho = R[name];
    }
    return 1;
}
//���������Ӳ�
double getdts(double* data, double t0, double t) {
    double dts;
    dts = data[1] + data[2] * (t - t0) + data[3] * (t - t0) * (t - t0);
    return dts;
}
double getdts(eph_t s, gtime_t rt) {
    double dts;
    dts = s.af0 + s.af1 * double(rt.time + rt.second - s.toc.time - s.toc.second) + s.af2 * double(rt.time + rt.second - s.toc.time - s.toc.second) * double(rt.time + rt.second - s.toc.time - s.toc.second);
    return dts;
}

void getsatelliteposition(eph_t satellite, char country, int name, double rho, double* xyz, double *dotxyz, gtime_t rt, gtime_t st0,double &tdts,double &tdtss) {
    struct COMMONTIME comtime;

    gtime_t tobs = rt;//�۲�ʱ��

    //�������ǹ��������
    double A = satellite.A;

    //����ƽ���˶����ٶ�
    double n0 = sqrt(GM / A / A / A);//GMΪ������������

    //������������ο���Ԫ��ʱ��

    double t = tobs.time - rho / clight - getdts(satellite, tobs) + satellite.tgd;
    double tk = t - satellite.toe.time;

    if (tk > 302400) tk = tk - 604800;
    else if (tk < -302400) tk = tk + 604800;
    else tk = tk;
    //��ƽ���˶����ٶȽ��и���Mk
    double n = satellite.deln + n0;
    //����ƽ���ǵ�
    double Mk = satellite.M0 + n * tk;
    //����ƫ���ǵ�
    double Ek = Mk;
    double Ek1;
    while (1) {
        //Ek1=Ek-(Ek-satellite.e*sin(Ek)-Mk)/(1-satellite.e*cos(Ek));
        Ek1 = Mk + satellite.e * sin(Ek);
        if (abs(Ek1 - Ek) < 1e-13) break;
        Ek = Ek1;
    }
    Ek = Ek1;
    //����������vk
    double vk = atan2(sqrt(1 - satellite.e * satellite.e) * sin(Ek) / (1 - satellite.e * cos(Ek)), (cos(Ek) - satellite.e) / (1 - satellite.e * cos(Ek)));
    //����������Ǿ�Phik
    double Phik = vk + satellite.omg;
    //������׵��͸�����
    double deltauk = satellite.cus * sin(2 * Phik) + satellite.cuc * cos(2 * Phik);
    double deltark = satellite.crs * sin(2 * Phik) + satellite.crc * cos(2 * Phik);
    double deltaik = satellite.cis * sin(2 * Phik) + satellite.cic * cos(2 * Phik);
    //��������������Ǿ�
    double uk = Phik + deltauk;
    //�����������
    double rk = A * (1 - satellite.e * cos(Ek)) + deltark;
    //��������Ĺ�����
    double ik = satellite.i0 + deltaik + satellite.idot * tk;
    //���������ڹ��ƽ���ϵ�λ��
    double xk1 = rk * cos(uk);
    double yk1 = rk * sin(uk);
    //���㾭�������������㾭��

    double Omegak = satellite.OMG0 + (satellite.OMGD - OMGE) * tk - OMGE * satellite.toes;
    //���������ڵ��ĵع�����ϵ��λ��
    double xk = xk1 * cos(Omegak) - yk1 * cos(ik) * sin(Omegak);
    double yk = xk1 * sin(Omegak) + yk1 * cos(ik) * cos(Omegak);
    double zk = yk1 * sin(ik);

    xyz[0] = xk;
    xyz[1] = yk;
    xyz[2] = zk;

    //����Ϊ���������ٶ�
    //����ƫ����ǵ�ʱ�䵼��
    double dotEk = n / (1 - satellite.e*cos(Ek));

    //���������ǵ�ʱ�䵼��
    double dotvk = sqrt((1+satellite.e)/(1-satellite.e))*cos(vk/2)*cos(vk/2)/cos(Ek/2)/cos(Ek/2)*dotEk;

    //���������Ǿ��ʱ�䵼��
    double dotuk = dotvk*(1+2*satellite.cus*cos(2*Phik)-2*satellite.cuc*sin(2*Phik));

    //�����򾶵�ʱ�䵼��
    double dotrk = dotEk * A * satellite.e * sin(Ek) + 2*dotvk * (satellite.crs*cos(2*Phik)-satellite.crc*sin(2*Phik));

    //��������ǵ�ʱ�䵼��
    double dotik = satellite.idot + 2 * dotvk * (satellite.cis*cos(2*Phik)-satellite.cic*sin(2*Phik));
    
    //�������ǹ��ƽ��λ�õ�ʱ�䵼��
    double dotxk = cos(uk) * dotrk - rk * sin(uk) * dotuk;
    double dotyk = sin(uk) * dotrk + rk * cos(uk) * dotuk;

    //���������ڵ�������ϵ�µ��ٶȲ�������������
    double dotR[12] = { cos(Omegak),-sin(Omegak) * cos(ik),-xk1 * sin(Omegak) - yk1 * cos(Omegak) * cos(ik), yk1 * sin(Omegak) * sin(ik),
                        sin(Omegak), cos(Omegak) * cos(ik), xk1 * cos(Omegak) - yk1 * sin(Omegak) * cos(ik),-yk1 * cos(Omegak) * sin(ik),
                        0          , sin(ik)              , 0                                            , yk*cos(ik)                 };
    double tx[4] = { dotxk,dotyk,satellite.OMGD - OMGE,dotik};
    double dotxyzr[3] = {};
    matx(dotR, 3, 4, tx, 4, 1, dotxyzr);
    dotxyz[0]=dotxyzr[0], dotxyz[1]=dotxyzr[1], dotxyz[2]=dotxyzr[2];
    //���������Ӳ���ٲ�������������
    tdts= getdts(satellite, tobs) + Fc * satellite.e * sqrt(satellite.A) * sin(Ek) - satellite.tgd;
    tdtss = satellite.af1 + 2*satellite.af2 * (tobs.time + tobs.second - tdts - satellite.toc.time - satellite.toc.second)+Fc*satellite.e*sqrt(A)*cos(Ek)*dotEk;
}


//���㶨λ�ô�������
double getR0(double* xyz0, double* xyz) {
    double R0;
    R0 = pow(xyz[0] - xyz0[0], 2) + pow(xyz[1] - xyz0[1], 2) + pow(xyz[2] - xyz0[2], 2);
    R0 = sqrt(R0);
    return R0;
}

double getl(double Xs, double X0, double R0) {
    return (Xs - X0) / R0;
}

double getm(double Ys, double Y0, double R0) {
    return (Ys - Y0) / R0;
}

double getn(double Zs, double Z0, double R0) {
    return (Zs - Z0) / R0;
}

void getmatrixL(double* Pi, double* R0, double* dts, double* dion, double* trp, int n, double* L) {
    int i;
    for (i = 0; i < n; i++) {
        L[i] = Pi[i] - R0[i] + clight * dts[i] - dion[i] - trp[i];
    }
}

void getmatrixB(double* l, double* m, double* n, int num, double* B) {
    int i;
    for (i = 0; i < num; i++) {
        B[4 * i + 0] = -l[i];
        B[4 * i + 1] = -m[i];
        B[4 * i + 2] = -n[i];
        B[4 * i + 3] = 1;
    }
}

void getmatrixP(double* var, int num, double* P) {
    for (int i = 0; i < num; i++)
        P[i * num + i] = 1.0 / var[i];
}

typedef struct {
    char sysytem[20] = { "GPS" };//Ĭ��ΪGPSϵͳ��������
    double xyzt[4] = {};//��λ���
    double xyztspeed[4] = {};//���ٽ��
    double PDOP = 0;//��λ��λ˥��
    double TDOP = 0;//ʱ���λ˥��
    double GDOP = 0;//���ε�λ˥��
    gtime_t tobs;
}SppResult;

SppResult get_SppResult(double Xr,double Yr,double Zr,double Dtr,double *dotxyzt,double pdot,double tdot,double gdot,gtime_t tobs) {
    SppResult Result;
    
    Result.xyzt[0] = Xr;Result.xyzt[1] = Yr;Result.xyzt[2] = Zr;Result.xyzt[3] = Dtr;

    Result.xyztspeed[0] = dotxyzt[0]; Result.xyztspeed[1] = dotxyzt[1]; 
    Result.xyztspeed[2] = dotxyzt[2]; Result.xyztspeed[3] = dotxyzt[3]/clight;
    //����clightֻ��Ҫ��һ�Σ�����������Ѿ�������˺����ھͲ��ô�����
    
    Result.PDOP = pdot, Result.TDOP = tdot, Result.GDOP = gdot;

    Result.tobs = tobs;
    
    //���������ı䶨λϵͳ���ƣ���Ȼ����Ĭ��Ϊ"GPS"
    
    return Result;
}
//���㶨λ������ղ���
/*��������*/
/*R��α��۲�ֵ�ṹ��*/
/*satllite�������ṹ������*/
/*ion��������ӳٲ�������*/
/*Result���������ṹ��*/

int SPPpos(GPSOBS R, eph_t* satllite, double* ion,SppResult &Result) {
    int num = R.num; double USEFUL[36] = {}; int usenum = 0; float Cno[36] = {}; float psrstd[36] = {}; float dopp[36] = {};
    int sname[36] = {};
    for (int i = 0; i < num; i++)
        if (R.name[i] <= 32&&satllite[R.name[i]].svh == 0&&satllite[R.name[i]].statu==EPHYES)
            USEFUL[usenum] = R.R0[i], sname[usenum] = R.name[i],Cno[usenum]=R.Cno[i],psrstd[usenum]=R.psrstd[i],dopp[usenum]=R.dopp[i], usenum++;
    num = usenum;

    //�����������
    if (num < 4) {
        printf("��������̫�٣��޷���λ");
        return 0;
    }

    //��ʼ���۲�ʱ��
    gtime_t tobs = R.rt;//�۲�ʱ��
    //��ʼ�����㶨λ�����ٱ���
    double dts[32] = {}, R0[32] = {}, l[32] = {}, m[32] = {}, n[32] = {}, xyz[3] = {}, Pi[32] = {}, xyz0[3] = { 100,100,100 },  dion[32] = {}, trp[32] = {}, var[32] = {};
    double dtss[32] = {}, dotxyz[3] = {}, w[32] = {};
    double PDOP, TDOP, GDOP, Xr, Yr, Zr, Dtr;
    while (1) {
        //������ȡ
        for (int i = 0; i < num; i++) {
            //��¼���Ǳ�Ų���ȡ��Ӧ���ǵ�����
            int name = sname[i];
            eph_t s = satllite[name];
            gtime_t st0 = s.toe;
            //��¼name[i]���ǵ�α��۲�ֵ
            Pi[i] = USEFUL[i]; double rho = Pi[i];
            //��ȡ����λ�����Ӳ�
            double tdts,tdtss;
            getsatelliteposition(s, 'G', name, rho, xyz, dotxyz,tobs, st0, tdts,tdtss);
            //��ȡname[i]�����ǵ��Ӳ�����
            dts[i] = tdts; dtss[i] = tdtss;
            //������ת����
            double alpha = OMGE * (rho / clight + dts[i]);
            xyz[0] = cos(alpha) * xyz[0] + sin(alpha) * xyz[1];
            xyz[1] = cos(alpha) * xyz[1] - sin(alpha) * xyz[0];
            //���㼸�ξ���R0����������
            R0[i] = getR0(xyz0, xyz);
            l[i] = getl(xyz[0], xyz0[0], R0[i]);
            m[i] = getm(xyz[1], xyz0[1], R0[i]);
            n[i] = getn(xyz[2], xyz0[2], R0[i]);
            //��������չ۲�ֵ
            double rorate = l[i] * dotxyz[0] + m[i] * dotxyz[1] + n[i] * dotxyz[2];
            w[i] = -0.19029367*dopp[i] - (rorate) + clight * dtss[i];//L1Ƶ�ʲ���19.03cm,L2Ƶ�ʲ���24.42cm,L5Ƶ�ʲ���25.48cm
            //��������,�������ӳ�
            double azel[2];
            getazel(xyz, xyz0, azel);
            struct GPSTime tobsgps; int week;
            tobsgps.Second = time2gpst(tobs, week);
            tobsgps.Week = week;
            dion[i] = ionmodel(tobsgps, ion, xyz0, azel);
            trp[i] = tropmodel(xyz0, azel);
            //����var(Ϊ����P������׼��)
            
            //var[i] = sqrt(7.2) + sqrt(abs(1.5 * dion[i]))  + sqrt(0.9 / (sin(abs(azel[1])) + 0.1)) + sqrt(2.7) + sqrt(0.009 / sin(abs(azel[1])));
            //var[i] = 0.004*0.004+0.003*0.003*cos(azel[1])*cos(azel[1]);//�߶Ƚ�ģ��1
            //var[i]= 0.004 * 0.004 + 0.003 * 0.003 / sin(azel[1]) /sin(azel[1]);//�߶Ƚ�ģ��2
            //var[i] = azel[1] >= pi / 6 ? 0.004 * 0.004 : 0.004 * 0.004 / 4 / sin(azel[1]) / sin(azel[1]);//�߶Ƚ�ģ��3
            //���ϸ߶ȽǶ�Ȩģ��Ӧ����Com_16�ļ�Ч�����У����е�һ����֣����ʦ��ԭ��γ����ṩ��Ч��Ҳ��̫�á�
            
            var[i] = exp(psrstd[i]);//α��۲�ֵ��׼��(ָ��������ʽ)
            //var[i] = (0.001 + psrstd[i]);//���Ժ�����ʽ
            //����α��۲�ֵ��׼��Ӧ����Com_16�ļ�Ч������Ϊ����,0927����(ָ�������Ժ�)
            //���������㷨�Ǵ��������Լ�������ģ�û���ϸ���������ݣ�ֻ���ض������ݹ��ã�����ʹ��
            
            //var[i] = 1;//��λ���ʽ
            //�����֮������
            
            //var[i] = 0.00224*pow(10, -Cno[i] / 10);//�����ģ��1���ز�L1����Ϊ0.00224��L2����Ϊ0.00077
            //���������ģ��Ӧ��0927�ļ�Ч����Ϊ����

            //0927�ļ���G10���������������Ⱥ�����Ӧ�������ģ�ͽ�����죬ֱ���ų�G10���Ի�ø����ȶ��Ľ��
            //Com_16�ļ���G08���ǶԼ������м����Ӱ�죬�����ų������Ͻ����Ϊ�ų�G08֮����
            /*������Ȩģ�ͻ�ӭ����*/
        }

        //������ƾ���
        double B[32 * 4] = { 0 };
        getmatrixB(l, m, n, num, B);

        //��Ϲ۲�ֵ����
        double L[32 * 1];
        getmatrixL(Pi, R0, dts, dion, trp, num, L);
        
        //Ȩ�ؾ������
        double P[32 * 32] = { 0 };//Ȩ�ؾ������
        getmatrixP(var, num, P);

        //�������Q
        double BT[32 * 4] = {};
        matT(B, num, 4, BT);
        double A[4 * 32] = {};
        matx(BT, 4, num, P, num, num, A);
        double Qr[4 * 4] = {};
        matx(A, 4, num, B, num, 4, Qr);
        double Q[4 * 4] = { 0 };
        inverseMatrix(Qr, 4, Q);

        //����λ�á��ٶȵ���С���˹���ֵ
        double X1[4 * 32] = {};
        matx(Q, 4, 4, BT, 4, num, X1);
        double X2[4 * 32] = {};
        matx(X1, 4, num, P, num, num, X2);
        double dxyzt[4] = {};
        matx(X2, 4, num, L, num, 1, dxyzt);//λ�ù���ֵ

        double dotxyzt[4] = {};
        matx(X2, 4, num, w, num, 1, dotxyzt);//�ٶȹ���ֵ
        
        //λ����ֵ����
        xyz0[0] = xyz0[0] + dxyzt[0];
        xyz0[1] = xyz0[1] + dxyzt[1];
        xyz0[2] = xyz0[2] + dxyzt[2];

        if (abs(dxyzt[0]) < 1e-4 && abs(dxyzt[1]) < 1e-4 && abs(dxyzt[2]) < 1e-4) {
            //���㾫����������ջ����ꡢ�ٶȡ��Ӳ����
            PDOP = sqrt(Q[0] + Q[5] + Q[10]);
            TDOP = sqrt(Q[15]);
            GDOP = sqrt(Q[0] + Q[5] + Q[10] + Q[15]);
            Xr = xyz0[0]; Yr = xyz0[1]; Zr = xyz0[2]; Dtr = dxyzt[3] / clight;
            Result=get_SppResult(Xr, Yr, Zr, Dtr, dotxyzt, PDOP, TDOP, GDOP,tobs);
            //printf("\n��������ղ��ٽ��%lf %lf %lf %lf\n", dotxyzt[0], dotxyzt[1], dotxyzt[2], dotxyzt[3]/clight);
            break;
        }
    }
    /*
    printf("��������");
    for (int j = 0; j < num; j++)
        printf("G%02d ", sname[j]);
    printf("��λ������£�\n");
    */
    return 1;
}

//������
void printResult(SppResult R,const char* mode="simple") {
    if ( strstr(mode,"simple"))
    {
        printf("%lld\n%lf\t%lf\t%lf\t%.6e\n", R.tobs.time, R.xyzt[0], R.xyzt[1], R.xyzt[2], R.xyzt[3]);
        printf("%lf\t%lf\t%lf\t%.6e\n", R.xyztspeed[0], R.xyztspeed[1], R.xyztspeed[2], R.xyztspeed[3]);
    }
    if (strstr(mode, "BLH"))
    {
        double blh[3] = {};
        xyztoblh(R.xyzt[0], R.xyzt[1], R.xyzt[2], blh);
        printf("%lld\n%lf\t%lf\t%lf\t%.6e\n", R.tobs.time, blh[0], blh[1], blh[2], R.xyzt[3]);
        printf("%lf\t%lf\t%lf\t%.6e\n", R.xyztspeed[0], R.xyztspeed[1], R.xyztspeed[2], R.xyztspeed[3]);
    }
    if (strstr(mode, "complex"))
    {
        double blh[3] = {};
        xyztoblh(R.xyzt[0], R.xyzt[1], R.xyzt[2], blh);
        printf("�۲�ʱ��:��׼����ʱ��%d��%d��%d��%d:%d:%.2lf\n", time2epoch(R.tobs).Year,
            time2epoch(R.tobs).Month,
            time2epoch(R.tobs).Day,
            time2epoch(R.tobs).Hour,
            time2epoch(R.tobs).Minute,
            time2epoch(R.tobs).Second);
        printf("X=%.6lf, Y=%.6lf, Z=%.6lf, ���ջ��Ӳ�Ϊ��%.6es\n", R.xyzt[0], R.xyzt[1], R.xyzt[2], R.xyzt[3]);
        printf("vX=%.6lf, vY=%.6lf, vZ=%.6lf, ���ջ�����Ϊ��%.6es\n", R.xyztspeed[0], R.xyztspeed[1], R.xyztspeed[2], R.xyztspeed[3]);
        printf("B=%lf,L=%lf,H=%lf\n", blh[0], blh[1], blh[2]);
        printf("��ά��λ����˥������PDOP=%.6lf\nʱ�侫��˥������TDOP=%.6lf\n���ξ���˥������GDOP=%.6lf\n", R.PDOP, R.TDOP, R.GDOP);
    }
}
//�������
void fprintResult(FILE* fp, SppResult R) {
    fprintf(fp,"%lld\t%lf\t%lf\t%lf\t%.6e\t ", R.tobs.time, R.xyzt[0], R.xyzt[1], R.xyzt[2], R.xyzt[3]);
    fprintf(fp,"%lf\t%lf\t%lf\t%.6e\n", R.xyztspeed[0], R.xyztspeed[1], R.xyztspeed[2], R.xyztspeed[3]);
}
void fprintResult(FILE* fp, SppResult R,const char *temp) {
    double blh[3] = {};
    xyztoblh(R.xyzt[0], R.xyzt[1], R.xyzt[2], blh);
    fprintf(fp, "%lld\t%lf\t%lf\t%lf\t%.6e\n", R.tobs.time, blh[0], blh[1], blh[2], R.xyzt[3]);
}
