#include <vector>


namespace icumsg
{
    class vehicle_state
    {
    public:
        int id;
        double vx;
        double vy;
        double h;
    };

    class POSITION
    {
    public:
        float x;

        float y;
    };
    
    class BOUNDINGBOX
    {
        public:
        icumsg::POSITION p1;

        icumsg::POSITION p2;

        icumsg::POSITION p3;

        icumsg::POSITION p4;

        int boxid;    //绑定box和id实验

    };
    class OBJECT
    {
    public:
        int id;

        int obj_type;

        float v;

        float theta;

        float width;

        float length;

        icumsg::BOUNDINGBOX corners;

        int pathNum;

        std::vector<POSITION> path;

        icumsg::POSITION p1;

        icumsg::POSITION p2;

        icumsg::POSITION p3;

        icumsg::POSITION p4;
    };
    class structFUSIONMAP
    {
    public:
        int timestamp;

        double utmX;

        double utmY;

        double mHeading;

        float resolution;

        int rows;

        int cols;

        int center_col;

        int center_row;

        int cells[401][151];
    };
    class structOBJECTLIST
    {
    public:
        int timestamp;

        int data_source;

        int count;

        std::vector<OBJECT> obj;

        // std::vector<sameLANE> sl;
    };
    
    
   
}

