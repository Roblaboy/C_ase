#include <iostream>
#include <map>
#include <vector>
#include <algorithm>
#include <tree_traverse/tree_traverse.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
using namespace std;
vector<int> query(map<int,int> &m, int p1, int p2)
{
    vector<int> line1,line2;
    line1.push_back(p1);
    line2.push_back(p2);
    while(m.count(p1))
    {
        line1.push_back(m[p1]);
        p1 = m[p1];
    }
    while(m.count(p2))
    {
        line2.push_back(m[p2]);
        p2 = m[p2];
    }
    reverse(line1.begin(), line1.end());//从根节点到尖端节点
    reverse(line2.begin(), line2.end());
    int i = 0,pos = 0;
    while(i < line1.size() && i < line2.size())
    {
        if(line1[i]== line2[i])
        {
            pos = i;
            i++;
        }
        else
            break;
    }
    //删除line1中第一个到pos处的所有元素，
    //删除line2中第一个到pos前一个的所有元素
    //将line1剩下的元素反转，拼接line1和line2
    std::vector<int>::iterator itor1;
    std::vector<int>::iterator itor2;
    itor1 = line1.begin() + pos;
    itor2 = line2.begin() + pos + 1;
    line1.erase(line1.begin(),itor1);
    line2.erase(line2.begin(),itor2);
    reverse(line1.begin(),line1.end());
    line1.insert(line1.end(), line2.begin(),line2.end());//line1中的元素从当前点到目标点的序列号依次排序
    return line1;
}