#include "../3rd/rapidjson/filereadstream.h"
#include "../3rd/rapidjson/document.h"
#include <iostream>
#include <fstream>
#include <string>
using namespace std;

namespace READFILE
{
    rapidjson::Document ReadJsonFile(const string &filepath)
    {
        rapidjson::Document doc;
        FILE *fp = fopen(filepath.c_str(), "r");
        if (fp == 0)
        {
            printf("[Error] cannot open %s.\n", filepath.c_str());
            return doc;
        }
        char read_buffer[2048];
        rapidjson::FileReadStream is(fp, read_buffer, sizeof(read_buffer));
        doc.ParseStream(is);
        fclose(fp);
        return doc;
    };
}