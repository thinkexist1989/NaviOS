#include "naviwidget.h"
#include "ui_naviwidget.h"
#include <QPainter>
#include <QMessageBox>
#include <QDebug>
extern ofstream fout;
NaviWidget::NaviWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::NaviWidget)
{
    i = 0;
    ui->setupUi(this);
    //ui->widget->setWindowTitle("NaviOS");
    compass.start();
    encoder.start();
    ultrasonic.start();
    voice.start();

    ultrasonicID = startTimer(200);
    locateID = startTimer(200);
    updateID = startTimer(100);

    fout.open("output.txt");
    usdata.open("usdata.txt");
    usdata << "GroupNum"<<"\t\t\t"<<"x"<<"\t\t\t"<<"y"<<"\t\t\t"<<"z"<<endl;

    locdata.open("locdata.txt");

    locdata<<"x"<<"\t\t\t"<<"y"<<"\t\t\t"<<"z"<<"\t\t\t"<<"Angle"<<endl;
    //ofstream fout("output.txt");
    //fout<<"x"<<"\t"<<"y"<<"\t"<<"z"<<"\t"<<"angle"<<endl;
}

NaviWidget::~NaviWidget()
{
    delete ui;
}

void NaviWidget::clearpluse()
{
    g_LeftPulse = 0;
    g_LeftDistance = 0;
    g_RightPulse = 0;
    g_RightDistance = 0;
}

void NaviWidget::timerEvent(QTimerEvent *event)
{

    if(event->timerId() == ultrasonicID){
        ultrasonic.SendLocateCommand(g_RcvNum);
       // ultrasonic.SendLocateCommand(1);
    }

    if(event->timerId() == locateID){

        fout << g_count << '\t' << g_RcvNumCnt << '\t' << g_dist[0] << "\t\t" << g_dist[1] << "\t\t" << g_dist[2] << "\t\t" << g_dist[3] << '\t'
             << g_Angle << "\t\t" <<  g_LeftDistance << "\t\t" << g_RightDistance << endl;
        g_count ++;
        input.Reload(g_LeftDistance, g_RightDistance);

        clearpluse();
        //printf("transation = %f", input.GetTranslation());
        state_rel.KalmanStateUpdate (&input);
        state_rel.KalmanCovUpdate (&input, &state_rel_update_uncertainty);
        kalman_gain_rel = state_rel.KalmanGain (&rel_obs_uncertainty, &obs_rel_matrix);

        g_us_mutex.lock();

        if(state_abs.GetX()>7572 && state_abs.GetX()<20472){
            if(g_radian>-1.57 && g_radian < 1.57)
                g_radian += 0.523;
            else if(g_radian<-2.2 || g_radian >2.2)
                    ;
        }

        MATRIX temp_matrix_1 = MATRIX (&g_radian, 1, 1);


        state_rel.KalmanStateCorrection (&temp_matrix_1,
                                         &kalman_gain_rel,
                                         &obs_rel_matrix,
                                         0);
        state_rel.KalmanCovCorrecction(&obs_rel_matrix, &kalman_gain_rel);
        //qDebug()<<state_rel.GetX ()<<"\t"<<state_rel.GetY ();
        delta = state_rel.GetDelta ();
        //qDebug()<<state_rel.last_state.GetX()<<"\t"<<state_rel.last_state.GetY();
         state_abs.KalmanStateUpdate (delta.GetX (), delta.GetY (), 0, 0);

         state_abs.KalmanCovUpdate (&state_abs_update_uncertainty);


         abs_obs = state_abs;



        if(g_NewLocMark == true){

            i++;

            Calculation(g_RcvNumCnt,g_ValidDist,g_ID,g_Group,abs_obs,state_rel,R_obs);

            g_NewLocMark = false;

            kalman_gain_abs = state_abs.KalmanGain (&R_obs, &obs_abs_matrix);
            vector_obs_abs[0] = abs_obs.GetX () + RELATIVENODE[g_GroupNum][RELATIVE_X];
            vector_obs_abs[1] = abs_obs.GetY () + RELATIVENODE[g_GroupNum][RELATIVE_Y];

            usdata<<g_GroupNum<<"\t\t"<<vector_obs_abs[0]<<"\t\t"<<vector_obs_abs[1]<<endl;

            MATRIX temp_matrix_2 = MATRIX (vector_obs_abs, 2, 1);
            state_abs.KalmanStateCorrection_abs (&temp_matrix_2,
                                                 &kalman_gain_abs,
                                                 &obs_abs_matrix,
                                                 abs_obs.GetZ (),
                                                 state_rel.GetOrientation());
            state_abs.KalmanCovCorrecction_abs (&obs_abs_matrix, &kalman_gain_abs);
        }

        locdata<<state_abs.GetX()<<"\t\t"<<state_abs.GetY()<<"\t\t"<<state_abs.GetZ()<<"\t\t"<<state_abs.GetOrientation()<<endl;


        //输出文本
        //ofstream fout("output.txt",ios::app|ios::ate);
        //fout<<state_abs.GetX()<<"\t"<<state_abs.GetY()<<"\t"<<state_abs.GetZ()<<"\t"<<state_abs.GetOrientation()<<endl;


        //超声波模块阵列号更改
        if(state_abs.GetX() >= 0 && state_abs.GetX() <= 7100){
            if(g_RcvNum != 0){
                g_RcvNum = 0;
            }
        }
        else if(state_abs.GetX() >= 7500 && state_abs.GetX() <= 17500){           
            if(g_RcvNum != 1){
                g_RcvNum = 1;
            }
        }
        else{
            if(g_RcvNum != 2){
                g_RcvNum = 2;
            }
        }

        g_us_mutex.unlock();



        //当前位置，重点位置播报
        if(state_abs.GetX() >= 300 && state_abs.GetX() <= 7100 && state_abs.GetY() >= 300 && state_abs.GetY() <= 5880){
            g_lastposition = g_currentposition;
            g_currentposition = IN_LAB1056;
        }
        else if(state_abs.GetX() >= 4000 && state_abs.GetX() <= 7500 && state_abs.GetY() >= 6080 && state_abs.GetY() <= 7580){
            g_lastposition = g_currentposition;
            g_currentposition = LAB1056;
        }
        else if(state_abs.GetX() >= 300 && state_abs.GetX() <= 1800 && state_abs.GetY() >= 6080 && state_abs.GetY() <= 9180){
            g_lastposition = g_currentposition;
            g_currentposition = LAB1057;
        }
        else if(state_abs.GetX() >=10700 && state_abs.GetX() <=14200 && state_abs.GetY() >= 6080 && state_abs.GetY() <= 7580){
            g_lastposition = g_currentposition;
            g_currentposition = LAB1055;
        }
        else if(state_abs.GetX() >=21750 && state_abs.GetX() <=25250 && state_abs.GetY() >= 6080 && state_abs.GetY() <= 7580){
            g_lastposition = g_currentposition;
            g_currentposition = LAB1050;
        }
        else if(state_abs.GetX() >=21400 && state_abs.GetX() <=25400 && state_abs.GetY() >= 8430 && state_abs.GetY() <= 10430){
            g_lastposition = g_currentposition;
            g_currentposition = DATING;
        }
        else{
            g_lastposition = g_currentposition;
            g_currentposition = ZOULANG;
        }

        if(g_currentposition != g_lastposition){
            if(g_currentposition != ZOULANG){
                if(voice.playingmark == true){
                    voice.playingmark = false;
                    voice.SendVoiceCommand(g_currentposition);
                    voice.playingmark = true;
                }
            }
        }
    }

    if(event->timerId() == updateID){
        ui->reckonAngle->setText(QString::number(state_rel.GetOrientation()/PI*180));
        ui->angle->setText(QString::number(g_Angle));
        ui->anglechange->setText(QString::number(g_Angle_Change));

        ui->X_Coor->setText(QString::number(state_abs.GetX()));
        ui->Y_Coor->setText(QString::number(state_abs.GetY()));
        ui->Z_Coor->setText(QString::number(state_abs.GetZ()));

        ui->cx->setText(QString::number(abs_obs.GetX()));
        ui->cy->setText(QString::number(abs_obs.GetY()));
        ui->cz->setText(QString::number(abs_obs.GetZ()));

        ui->GroupNum->setText(QString::number(g_GroupNum));
        ui->RcvNumCnt->setText(QString::number(g_RcvNumCnt));
        ui->RcvNum->setText(QString::number(g_RcvNum));

        ui->leftpluse->setText(QString::number(g_LeftPulseDisplay));
        ui->rightpluse->setText(QString::number(g_RightPulseDisplay));

        ui->voiceorder->setText(QString::number(g_VoiceOrder));
        ui->playorder->setText(QString::number(g_PlayOrder));

        ui->inti->setText(QString::number(i));

        update();
    }
}

void NaviWidget::paintEvent(QPaintEvent *event)
{
   QPainter painter;
   QPixmap map("map.jpg");
   painter.begin(&map);
   painter.save();
   painter.setBrush(Qt::red);
   painter.scale((float)500/27650,(float)300/16580);
   painter.drawEllipse(state_abs.GetX(),state_abs.GetY(),500,500);
   painter.restore();
   painter.end();

   painter.begin(this);
   painter.drawPixmap(230,40,map);

}

void NaviWidget::on_setzero_clicked()
{
    compass.terminate();

    compass.SendAngleCommand(SET_ZERO);
    compass.start();
}

void NaviWidget::on_calibration_clicked()
{
    compass.terminate();
    compass.SendAngleCommand(START_CALIBRATION);

    int ret = QMessageBox::information(this,tr("Calibration"),tr("Calibrating..."),QMessageBox::Ok);

    if(ret == QMessageBox::Ok){
        compass.SendAngleCommand(STOP_CALIBRATION);
        compass.start();
    }
}
