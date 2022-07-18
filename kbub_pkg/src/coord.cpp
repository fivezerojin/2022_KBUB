#include <iostream>
#include <fstream>
#include <string>
#define NUM_OF_TXT 110//txt파일의 개수
using namespace std;
class listCoord
{
    public:
    double coordList[NUM_OF_TXT][20000][2]; //경로가 저장되는 배열([0]->x, [1] -> y)
    int n[NUM_OF_TXT];
    int count = 1;
    int end = NUM_OF_TXT;
    std::string k;
    listCoord()
    {
        for(int  i = 0; i < NUM_OF_TXT; i++)
            n[i] = 0;
        for(int k = 0; k <NUM_OF_TXT; k++)
        {
            for(int i = 0; i < 20000; i++)
            {
                for(int j = 0; j < 2; j++)
                {
                    coordList[k][i][j] = 1.10;
                }
            }
        }
    }
    void readText()
    {
        ifstream ifile;
        char line[5000];
        string file = "/home/usera/catkin_ws/src/kbub_pkg/src/map/";//맵파일 경로

        
        for(count = 1; count <= end; count++)
        {
            
            ifile.open(file + to_string(count)+".txt");
            if(ifile.is_open())
            {
                while(ifile.getline(line, sizeof(line)))
                {
                    char *cx = strtok(line, ",");
                    char *cy;
                    
                    while(cx != NULL)
                    {
                        coordList[count][n[count]][0] = atof(cx);
                        cy = strtok(NULL, " ");
                        cx = strtok(NULL, ",");
                        coordList[count][n[count]++][1] = atof(cy);
                    }
                }
            }
            ifile.close();
        }
    }
};