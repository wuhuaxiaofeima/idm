// ExampleConsole.cpp : Defines the entry point for the console application.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include "./../3rd/RDBHandler/RDBHandler.hh"
#include "data_defination.hpp"
#include <vector>
#include <functional>
#include <iostream>
#include <algorithm>
#include "PIDController/PIDController.h"
#include "./../cfg/READCONFIG.hpp"

const static string CONFIG_PATH = "./../cfg/config.json";
auto CONFIG=READFILE::ReadJsonFile(CONFIG_PATH); //when deploy to another target, should use absolute path.
const string PID_PARAMETER_PATH=CONFIG["PID_PARAMETER_PATH"].GetString();
// const string ZCM_URL=CONFIG["ZCM_URL"].GetString();
// bool tievflag=CONFIG["CONNECT_TIEV"].GetBool();

#define DEFAULT_PORT 48190
#define DEFAULT_BUFFER 204800
#define OBJECTLIST_PATH_NUM 5



using namespace std;
// const static string CONFIG_PATH = "../cfg/config.json";
// auto CONFIG=READFILE::ReadJsonFile(CONFIG_PATH); //when deploy to another target, should use absolute path.
// const string PID_PARAMETER_PATH=CONFIG["PID_PARAMETER_PATH"].GetString();

char szServer[128];       // Server to connect to
int iPort = DEFAULT_PORT; // Port on server to connect to
int sClient;

static RDB_OBJECT_STATE_t sOwnObjectState; // own object's state
static double wheelbase = 3;
static PIDController pid_controller(PID_PARAMETER_PATH); 
int egoroad = 0; 
int egolane = 0; 

int egoId = 1;//选择需要控制的车辆ID！！

double ego_vx;
double ego_vy;
double ego_h;
double ego_speed_h;
double hdgRel;
double ACC;
double _step_count = 0;

vector<icumsg::OBJECT> objlist;
vector<int> samelane;
icumsg::structFUSIONMAP fumap;
vector<icumsg::vehicle_state> vhss;

void parseRDBMessage(RDB_MSG_t *msg, bool &isImage);
void parseRDBMessageEntry(const double &simTime, const unsigned int &simFrame, RDB_MSG_ENTRY_HDR_t *entryHdr);
void sendDriverCtrl(int &sendSocket, const double &simTime, const unsigned int &simFrame);
void packEgoVehFusionMapBase(const double &simTime, RDB_OBJECT_STATE_t &item); //地图初始化
void packEgoVehObjectList(RDB_OBJECT_STATE_t &item);                           //对象列表
void handleRDBitem_OBJ_STATE(const double &simTime, const unsigned int &simFrame, RDB_OBJECT_STATE_t &item, bool isExtended); //
void lanedetection(RDB_ROAD_POS_t &item);                                                                                     //
double FindingFrontVehicle_And_CalAcc();
void fusionmapfill();
double IDM(double EGO_V, double FRONT_V, double FRONTDIS, int FLAG);

inline void projectToVehFrame(double x, double y, double &tarx, double &tary)
{
    //  now we don't consider the Z direction.
    double offset = wheelbase - 1.48;
    double heading = sOwnObjectState.base.pos.h;
    double xr = x - sOwnObjectState.base.pos.x;
    double yr = y - sOwnObjectState.base.pos.y;
    tarx = sin(heading) * xr - cos(heading) * yr;
    tary = cos(heading) * xr + sin(heading) * yr - offset;
}

inline bool isInEgoVehFrame(double tarx, double tary, double tarz)
{
    // check whether target is within ego car's frame coordinate.
    // frame: ego car's position as origin, heading direction as y, righthand side as x.
    // forwards 60m, backwards 20m, left & right side each 15m
    // considering cars' length and width, leave some margin.
    double tarz_veh_frame = tarz - sOwnObjectState.base.pos.z;
    double tarx_veh_frame;
    double tary_veh_frame;
    projectToVehFrame(tarx, tary, tarx_veh_frame, tary_veh_frame);
    return (tary_veh_frame < 63) && (tary_veh_frame > -23) && (fabs(tarx_veh_frame) < 17) && (tarz_veh_frame < 2);
}
inline bool testInArea(double x, double y, const icumsg::BOUNDINGBOX &box)
{
    // only need to calculate the z dimension of vectors' cross products.
    // if all cross products have the same symbol, the point is located within the area.
    double z1 = (box.p2.x - box.p1.x) * (y - box.p1.y) - (box.p2.y - box.p1.y) * (x - box.p1.x);
    bool isNeg = z1 < 0;
    double z2 = (box.p3.x - box.p2.x) * (y - box.p2.y) - (box.p3.y - box.p2.y) * (x - box.p2.x);
    if ((isNeg && z2 >= 0) || (!isNeg && z2 < 0))
        return false; // once result's symbol is different from previous ones, return false.
    double z3 = (box.p4.x - box.p3.x) * (y - box.p3.y) - (box.p4.y - box.p3.y) * (x - box.p3.x);
    if ((isNeg && z3 >= 0) || (!isNeg && z3 < 0))
        return false;
    double z4 = (box.p1.x - box.p4.x) * (y - box.p4.y) - (box.p1.y - box.p4.y) * (x - box.p4.x);
    if ((isNeg && z4 >= 0) || (!isNeg && z4 < 0))
        return false;
    return true;
}
void usage()
{
    printf("usage: client [-p:x] [-s:IP]\n\n");
    printf("       -p:x      Remote port to send to\n");
    printf("       -s:IP     Server's IP address or hostname\n");
    exit(1);
}

//
// Function: ValidateArgs
//
// Description:
//    Parse the command line arguments, and set some global flags
//    to indicate what actions to perform
//
void ValidateArgs(int argc, char **argv)
{
    int i;

    strcpy(szServer, "127.0.0.1");

    for (i = 1; i < argc; i++)
    {
        if ((argv[i][0] == '-') || (argv[i][0] == '/'))
        {
            switch (tolower(argv[i][1]))
            {
            case 'p': // Remote port
                if (strlen(argv[i]) > 3)
                    iPort = atoi(&argv[i][3]);
                break;
            case 's': // Server
                if (strlen(argv[i]) > 3)
                    strcpy(szServer, &argv[i][3]);
                break;
            default:
                usage();
                break;
            }
        }
    }
}

int main(int argc, char *argv[])
{
    char *szBuffer = new char[DEFAULT_BUFFER]; // allocate on heap
    int ret;

    struct sockaddr_in server;
    struct hostent *host = NULL;

    static bool sSendData = false;
    static bool sVerbose = true;
    static bool sSendTrigger = false;

    // Parse the command line
    //
    ValidateArgs(argc, argv);

    //
    // Create the socket, and attempt to connect to the server
    //
    sClient = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    if (sClient == -1)
    {
        fprintf(stderr, "socket() failed: %s\n", strerror(errno));
        return 1;
    }

    int opt = 1;
    setsockopt(sClient, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));

    server.sin_family = AF_INET;
    server.sin_port = htons(iPort);
    server.sin_addr.s_addr = inet_addr(szServer);

    //
    // If the supplied server address wasn't in the form
    // "aaa.bbb.ccc.ddd" it's a hostname, so try to resolve it
    //
    if (server.sin_addr.s_addr == INADDR_NONE)
    {
        host = gethostbyname(szServer);
        if (host == NULL)
        {
            fprintf(stderr, "Unable to resolve server: %s\n", szServer);
            return 1;
        }
        memcpy(&server.sin_addr, host->h_addr_list[0], host->h_length);
    }
    // wait for connection
    bool bConnected = false;

    while (!bConnected)
    {
        if (connect(sClient, (struct sockaddr *)&server, sizeof(server)) == -1)
        {
            fprintf(stderr, "connect() failed: %s\n", strerror(errno));
            sleep(1);
        }
        else
            bConnected = true;
    }

    fprintf(stderr, "connected!\n");

    unsigned int bytesInBuffer = 0;
    size_t bufferSize = sizeof(RDB_MSG_HDR_t);
    unsigned int count = 0;
    unsigned char *pData = (unsigned char *)calloc(1, bufferSize);

    // Send and receive data - forever!
    //
    for (;;)
    {
        bool bMsgComplete = false;

        // read a complete message
        while (!bMsgComplete)
        {
            // if ( sSendTrigger && !( count++ % 1000 ) )
            //   sendTrigger( sClient, 0, 0 );

            ret = recv(sClient, szBuffer, DEFAULT_BUFFER, 0);

            if (ret == -1)
            {
                printf("recv() failed: %s\n", strerror(errno));
                break;
            }

            if (ret != 0)
            {
                // do we have to grow the buffer??
                if ((bytesInBuffer + ret) > bufferSize)
                {
                    pData = (unsigned char *)realloc(pData, bytesInBuffer + ret);
                    bufferSize = bytesInBuffer + ret;
                }

                memcpy(pData + bytesInBuffer, szBuffer, ret);
                bytesInBuffer += ret;

                // already complete messagae?
                if (bytesInBuffer >= sizeof(RDB_MSG_HDR_t))
                {
                    RDB_MSG_HDR_t *hdr = (RDB_MSG_HDR_t *)pData;

                    // is this message containing the valid magic number?
                    if (hdr->magicNo != RDB_MAGIC_NO)
                    {
                        printf("message receiving is out of sync; discarding data");
                        bytesInBuffer = 0;
                    }

                    while (bytesInBuffer >= (hdr->headerSize + hdr->dataSize))
                    {
                        unsigned int msgSize = hdr->headerSize + hdr->dataSize;
                        bool isImage = false;

                        // print the message
                        // if ( sVerbose )
                        //    Framework::RDBHandler::printMessage( ( RDB_MSG_t* ) pData, true );

                        // now parse the message
                        parseRDBMessage((RDB_MSG_t *)pData, isImage);

                        // remove message from queue
                        memmove(pData, pData + msgSize, bytesInBuffer - msgSize);
                        bytesInBuffer -= msgSize;

                        bMsgComplete = true;
                    }
                }
            }
        }
        // do some other stuff before returning to network reading
    }
    ::close(sClient);

    return 0;
}

void parseRDBMessage(RDB_MSG_t *msg, bool &isImage)
{
    if (!msg)
        return;

    if (!msg->hdr.dataSize)
        return;

    RDB_MSG_ENTRY_HDR_t *entry = (RDB_MSG_ENTRY_HDR_t *)(((char *)msg) + msg->hdr.headerSize);
    uint32_t remainingBytes = msg->hdr.dataSize;

    while (remainingBytes)
    {
        parseRDBMessageEntry(msg->hdr.simTime, msg->hdr.frameNo, entry);

        isImage |= (entry->pkgId == RDB_PKG_ID_IMAGE);

        remainingBytes -= (entry->headerSize + entry->dataSize);

        if (remainingBytes)
            entry = (RDB_MSG_ENTRY_HDR_t *)((((char *)entry) + entry->headerSize + entry->dataSize));
    }
}

void parseRDBMessageEntry(const double &simTime, const unsigned int &simFrame, RDB_MSG_ENTRY_HDR_t *entryHdr)
{
    if (!entryHdr)
        return;

    int noElements = entryHdr->elementSize ? (entryHdr->dataSize / entryHdr->elementSize) : 0;

    if (!noElements) // some elements require special treatment
    {
        switch (entryHdr->pkgId)
        {
        case RDB_PKG_ID_START_OF_FRAME:
            objlist.clear();
            samelane.clear();
            vhss.clear();
            for (int y = 0; y < 401; y++) //横向上没必要0~150宽
            {
                for (int x = 0; x < 151; x++)
                {
                    fumap.cells[y][x]=0;
                }
            }
            fprintf(stderr, "void parseRDBMessageEntry: got start of frame\n");
            break;

        case RDB_PKG_ID_END_OF_FRAME:

            //FindingFrontVehicle_And_CalAcc();
            ACC = FindingFrontVehicle_And_CalAcc();
            fprintf(stderr, "void parseRDBMessageEntry: got end of frame\n");

            sendDriverCtrl(sClient, simTime, simFrame);
            break;

        default:
            return;
            break;
        }
        return;
    }

    unsigned char ident = 6;
    char *dataPtr = (char *)entryHdr;

    dataPtr += entryHdr->headerSize;

    while (noElements--)
    {
        bool printedMsg = true;
        switch (entryHdr->pkgId)
        {
        case RDB_PKG_ID_ROAD_POS:

            lanedetection(*((RDB_ROAD_POS_t *)dataPtr));

            break;
        case RDB_PKG_ID_OBJECT_STATE:

            handleRDBitem_OBJ_STATE(simTime, simFrame, *((RDB_OBJECT_STATE_t *)dataPtr), entryHdr->flags & RDB_PKG_FLAG_EXTENDED);

            break;
        default:
            printedMsg = false;
            break;
        }

        dataPtr += entryHdr->elementSize;
    }
}

void lanedetection(RDB_ROAD_POS_t &item) //判断是否同一车道
{
    if (item.playerId == egoId)
    {
        egolane = item.laneId;
        egoroad = item.roadId;
        hdgRel = item.hdgRel;
        // fprintf( stderr, " hdgRel = %lf\n" , hdgRel);
    }
    else if (abs(item.laneId) == abs(egolane) && item.playerId != egoId)
    {
        samelane.push_back(item.playerId);
    }
}

void packEgoVehObjectList(RDB_OBJECT_STATE_t &item)
{

    double speed = sqrt(pow(item.ext.speed.x, 2) + pow(item.ext.speed.y, 2));

    icumsg::OBJECT obj;

    obj.id = item.base.id;
    switch (item.base.type)
    {
    case RDB_OBJECT_TYPE_PLAYER_CAR:
        obj.obj_type = 0; // 0 for vehicle
        break;
    case RDB_OBJECT_TYPE_PLAYER_BUS:
        obj.obj_type = 0; // 0 for vehicle
        break;
    case RDB_OBJECT_TYPE_PLAYER_TRUCK:
        obj.obj_type = 0; // 0 for vehicle
        break;
    case RDB_OBJECT_TYPE_PLAYER_VAN:
        obj.obj_type = 0; // 0 for vehicle
        break;
    case RDB_OBJECT_TYPE_PLAYER_TRAILER:
        obj.obj_type = 0; // 0 for vehicle
        break;
    case RDB_OBJECT_PLAYER_SEMI_TRAILER:
        obj.obj_type = 0; // 0 for vehicle
        break;
    // maybe TiEV can't consider railcar, however I stilled put railcar here as a dynamic object.
    case RDB_OBJECT_PLAYER_RAILCAR:
        obj.obj_type = 0; // 0 for vehicle
        break;
    case RDB_OBJECT_PLAYER_RAILCAR_SEMI_HEAD:
        obj.obj_type = 0; // 0 for vehicle
        break;
    case RDB_OBJECT_PLAYER_RAILCAR_SEMI_BACK:
        obj.obj_type = 0; // 0 for vehicle
        break;
    case RDB_OBJECT_TYPE_PLAYER_BIKE:
        obj.obj_type = 1; // 1 for bicyclist
        break;
    case RDB_OBJECT_TYPE_PLAYER_MOTORBIKE:
        obj.obj_type = 1; // 1 for bicyclist
        break;
    case RDB_OBJECT_TYPE_PLAYER_PEDESTRIAN:
        obj.obj_type = 2; // 2 for pedestrain
        break;
    case RDB_OBJECT_TYPE_PLAYER_PED_GROUP:
        obj.obj_type = 2; // 2 for pedestrain
        break;
    default:
        obj.obj_type = 127; // others
    }
    obj.v = speed; // not sure absolute or relative, now is absolute speed.
    obj.length = item.base.geo.dimX;
    obj.width = item.base.geo.dimY;
    double theta_r = item.base.pos.h - sOwnObjectState.base.pos.h + M_PI / 2;
    if (theta_r > M_PI)
        theta_r = theta_r - 2 * M_PI;
    else if (theta_r < -M_PI)
        theta_r = theta_r + 2 * M_PI;
    obj.theta = theta_r;

    obj.pathNum = OBJECTLIST_PATH_NUM;
    std::vector<icumsg::POSITION> center_points(obj.pathNum); //
    icumsg::POSITION abs_center;
    // vtd ground coords is different from veh coords
    abs_center.x = item.base.pos.x + cos(item.base.pos.h) * item.base.geo.offX - sin(item.base.pos.h) * item.base.geo.offY;
    abs_center.y = item.base.pos.y + sin(item.base.pos.h) * item.base.geo.offX + cos(item.base.pos.h) * item.base.geo.offY;

    for (int i = 0; i < obj.pathNum; ++i) //可注释掉该函数？
    {
        double x, y;
        x = abs_center.x + i * item.ext.speed.x + 0.5 * i * i * item.ext.accel.x;
        y = abs_center.y + i * item.ext.speed.y + 0.5 * i * i * item.ext.accel.y;
        projectToVehFrame(x, y, x, y);
        center_points[i].x = x;
        center_points[i].y = y;
    }
    // center_points[0].x = abs_center.x;
    // center_points[0].y = abs_center.y;

    obj.path = center_points;

    std::vector<icumsg::POSITION> corner_points(4);

    corner_points[0].x = center_points[0].x + 0.5 * obj.length * cos(obj.theta) + 0.5 * obj.width * sin(obj.theta);
    corner_points[0].y = center_points[0].y + 0.5 * obj.length * sin(obj.theta) - 0.5 * obj.width * cos(obj.theta);
    corner_points[1].x = center_points[0].x - 0.5 * obj.length * cos(obj.theta) + 0.5 * obj.width * sin(obj.theta);
    corner_points[1].y = center_points[0].y - 0.5 * obj.length * sin(obj.theta) - 0.5 * obj.width * cos(obj.theta);
    corner_points[2].x = center_points[0].x - 0.5 * obj.length * cos(obj.theta) - 0.5 * obj.width * sin(obj.theta);
    corner_points[2].y = center_points[0].y - 0.5 * obj.length * sin(obj.theta) + 0.5 * obj.width * cos(obj.theta);
    corner_points[3].x = center_points[0].x + 0.5 * obj.length * cos(obj.theta) - 0.5 * obj.width * sin(obj.theta);
    corner_points[3].y = center_points[0].y + 0.5 * obj.length * sin(obj.theta) + 0.5 * obj.width * cos(obj.theta);
    for (int i = 0; i < 4; ++i)
    {
        if (corner_points[i].x <= center_points[0].x)
        {
            if (corner_points[i].y <= center_points[0].y)
                obj.corners.p3 = corner_points[i];
            else
                obj.corners.p4 = corner_points[i];
        }
        else
        {
            if (corner_points[i].y <= center_points[0].y)
                obj.corners.p2 = corner_points[i];
            else
                obj.corners.p1 = corner_points[i];
        }
    }

    obj.corners.boxid = item.base.id;
    objlist.push_back(obj);
}

void fusionmapfill() //基于所有对象填充地图
{
    //每个对象依次填充
    if (objlist.empty())
    {
        return;
    }
    icumsg::BOUNDINGBOX box = objlist.back().corners;

    // fprintf( stderr, " boxid = %d\n" , box.boxid);

    // find sampling area
    double left = box.p3.x < box.p4.x ? box.p3.x : box.p4.x;  //比较p3p4，选取更小的
    double right = box.p1.x > box.p2.x ? box.p1.x : box.p2.x; //比较p1p2，选取更大的
    double bottom = box.p2.y < box.p3.y ? box.p2.y : box.p3.y;
    double top = box.p1.y > box.p4.y ? box.p1.y : box.p4.y;
    int left_cell = floor(left / 0.2) + 75;
    int right_cell = ceil(right / 0.2) + 75;
    int bottom_cell = 300 - floor(bottom / 0.2);
    int top_cell = 300 - ceil(top / 0.2);
    int xmin = left_cell > -1 ? left_cell : 0;
    int xmax = right_cell < 151 ? right_cell : 150;
    int ymin = top_cell > -1 ? top_cell : 0;
    int ymax = bottom_cell < 401 ? bottom_cell : 400;
    // fill map cells
    int x = xmin;
    int y = ymin;
    while (x <= xmax)
    {
        y = ymin;
        while (y <= ymax)
        {
            if (testInArea((x - 75) * 0.2, (300 - y) * 0.2, box))
            {
                fumap.cells[y][x] = box.boxid;
            }
            ++y;
        }
        ++x;
    }
}

double FindingFrontVehicle_And_CalAcc() //基于地图，找前车
{
    int max_y = 0;
    int front_id = 0;
    double FRONT_V;
    int has_front = 0;
    double frontdis = 60;

    for (int y = 0; y < 300; y++) //横向上没必要0~150宽
    {
        for (int x = 65; x < 85; x++)
        {
            // cout<<fumap.cells[y][x]<<" ";
        }
        // cout<<endl;
    }
    for (int x = 65; x < 85; x++) //横向上没必要0~150宽
    {
        for (int y = 0; y < 300; y++)
        {
            int boxid = fumap.cells[y][x];
            vector<int>::iterator result = find(samelane.begin(), samelane.end(), boxid);

            if (result != samelane.end() && boxid > 1 && y > max_y) //遍历同车道
            {
                for (int i = 0; i < vhss.size(); i++) //遍历对象状态
                {
                    icumsg::vehicle_state vs = vhss[i];
                    if (vs.id == boxid && vs.vx * ego_vx + vs.vy * ego_vy > 0) //相同方向
                    {
                        max_y = y;
                        front_id = boxid;
                        FRONT_V = sqrt(vs.vx * vs.vx + vs.vy * vs.vy);
                        frontdis = (300 - max_y) * 0.2;
                        has_front = 1;
                    }
                }
            }
        }
    }
    cout << "dis" << frontdis << endl;
    cout << "id" << front_id << endl;
    double ego_v = sqrt(ego_vx * ego_vx + ego_vy * ego_vy);
    double A = IDM(ego_v, FRONT_V, frontdis, has_front);
    return A;
}

void packEgoVehFusionMapBase(const double &simTime, RDB_OBJECT_STATE_t &item)
{

    fumap.timestamp = simTime * 1000; // s -> ms
    // zcm_manager.fusionmap_.utmX = zcm_manager.navinfo_.utmX;
    // zcm_manager.fusionmap_.utmY = zcm_manager.navinfo_.utmY;
    // zcm_manager.fusionmap_.mHeading = zcm_manager.navinfo_.mHeading;  //导航地图方向如何表示？？
    fumap.rows = 401;
    fumap.cols = 151;
    fumap.center_row = 300;
    fumap.center_col = 75;
    fumap.resolution = 0.2;
}

void handleRDBitem_OBJ_STATE(const double &simTime, const unsigned int &simFrame, RDB_OBJECT_STATE_t &item, bool isExtended)
{
    if (item.base.id == egoId) //自车
    {
        packEgoVehFusionMapBase(simTime, item);
        sOwnObjectState = item; // cache ego car's state
        ego_vx = item.ext.speed.x;
        ego_vy = item.ext.speed.y;
        ego_h = item.ext.speed.h;
        ego_speed_h = item.base.pos.h;
        // fprintf(stderr, "ego_vx =%lf\n" , ego_vx);
        // fprintf(stderr, "ego_h =%lf\n" , ego_h);
        // fprintf(stderr, "ego_speed_h =%lf\n" , ego_speed_h);

    }
    else
    {
        icumsg::vehicle_state stat;
        stat.id = item.base.id;
        stat.vx = item.ext.speed.x;
        stat.vy = item.ext.speed.y;
        stat.h = item.ext.speed.h;
        vhss.push_back(stat);
        packEgoVehObjectList(item);
        fusionmapfill();
    }
}

double IDM(double EGO_V, double FRONT_V, double FRONTDIS, int FLAG)
{
    double ACC_MAX = 6.0;
    double COMFORT_ACC_MAX = 3.0;
    double COMFORT_ACC_MIN = -5.0;
    double LENGTH = 0;
    double S0 = 5.0;              
    double DISTANCE_WANTED = 5.0; 
    double TIME_WANTED = 1.5;
    double DELTA = 4.0;
    double SPEED_DIFFERENCE = EGO_V - FRONT_V;
    double MAX = 0.0;
    double EGO_TARGET_SPEED = 0.0;
    double TARGET_SPEED = 30.0;
    double DESIRED_GAP = 0;
    // ego_target_speed = abs(utils.not_zero(getattr(ego_vehicle, "target_speed", 0)))

    // acceleration = self.COMFORT_ACC_MAX * (
    //                 1 - np.power(max(ego_vehicle.speed, 0) / ego_target_speed, self.DELTA))
   
    
    ACC = COMFORT_ACC_MAX * (1 - pow((EGO_V / TARGET_SPEED), DELTA));
    
    if ((FLAG != 0))
    {
        DESIRED_GAP = DISTANCE_WANTED + EGO_V * TIME_WANTED + EGO_V * SPEED_DIFFERENCE / (2 * sqrt(-COMFORT_ACC_MAX * COMFORT_ACC_MIN)); //此处a和知乎不一致
        // ACC = ACC_MAX * (1 - pow((EGO_V/TARGET_SPEED), DELTA) - pow(DISTANCE_WANTED/FRONTDIS, 2));//期望车速没有
        // ACC -= COMFORT_ACC_MAX * pow(DESIRED_GAP / FRONTDIS, 2);
         ACC -= COMFORT_ACC_MAX * pow(DESIRED_GAP / FRONTDIS, 2);
        // fprintf(stderr, "DESIRED_GAP =%lf\n" , DESIRED_GAP);
        // fprintf(stderr, "FRONTDIS =%lf\n" , FRONTDIS);
    }
    return ACC;
}


void sendDriverCtrl(int &sendSocket, const double &simTime, const unsigned int &simFrame)
{
    Framework::RDBHandler myHandler;

    myHandler.initMsg();

    RDB_DRIVER_CTRL_t *myDriver = (RDB_DRIVER_CTRL_t *)myHandler.addPackage(simTime, simFrame, RDB_PKG_ID_DRIVER_CTRL);

    if (!myDriver)
        return;
    double aim_speed = ACC + ego_vx;
    double aim_steer = hdgRel;//tick需要输入两个值，hdgRel本就是差值，所以另一个赋值为零
    double veh_speed = ego_vx;
    double veh_steer = 0;
    fprintf(stderr, "aim_steer%lf\n" , aim_steer);
    fprintf(stderr, "veh_steer%lf\n" , veh_steer);
    pid_controller.tick(aim_speed, aim_steer, veh_speed , veh_steer);

    // myDriver->throttlePedal = pid_controller.control.throttle;
    // myDriver->brakePedal = pid_controller.control.brake;
    myDriver->playerId = egoId;
    // myDriver->accelTgt = 5;
    // myDriver->throttlePedal = pid_controller.control.throttle;
    // myDriver->brakePedal = pid_controller.control.brake;
    //  fprintf(stderr, "=======acc===%lf\n" , ACC);
    myDriver->accelTgt  = ACC;
    fprintf(stderr, "pid_controller.control.steer===%lf\n" , pid_controller.control.steer);
    myDriver->steeringTgt = pid_controller.control.steer * M_PI / 180;
    // myDriver->steeringTgt=0;  //单位弧度，正值向左转
    // myDriver->steeringTorque = 2.0;
    // myDriver->validityFlags = RDB_DRIVER_INPUT_VALIDITY_TGT_ACCEL | RDB_DRIVER_INPUT_VALIDITY_TGT_STEERING | RDB_DRIVER_INPUT_VALIDITY_ADD_ON;
    myDriver->validityFlags = RDB_DRIVER_INPUT_VALIDITY_TGT_ACCEL | RDB_DRIVER_INPUT_VALIDITY_TGT_STEERING | RDB_DRIVER_INPUT_VALIDITY_ADD_ON;
    // myDriver->validityFlags = RDB_DRIVER_INPUT_VALIDITY_THROTTLE | RDB_DRIVER_INPUT_VALIDITY_BRAKE | RDB_DRIVER_INPUT_VALIDITY_STEERING_TORQUE |RDB_DRIVER_INPUT_VALIDITY_ADD_ON;
    int retVal = send(sendSocket, (const char *)(myHandler.getMsg()), myHandler.getMsgTotalSize(), 0);

    if (!retVal)
        fprintf(stderr, "sendDriverCtrl: could not send trigger\n");
}
