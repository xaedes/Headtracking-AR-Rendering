#include <iterator>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

namespace ar{
namespace common{

class CSVRow
{
public:
    CSVRow();
    CSVRow(char seperator);

    std::string const& operator[](std::size_t index) const;
    std::size_t size() const;

    template<typename T>
    T getAs(std::size_t index) const
    {
        std::stringstream sstream(m_data[index]);
        T result;
        sstream >> result;
        return result;
    }

    void readNextRow(std::istream& str);
    int find(const std::string& string);
    int find(const std::string& string, const std::string& prefix, const std::string& suffix);


private:
    char m_seperator;
    std::vector<std::string> m_data;
};

std::istream& operator>>(std::istream& str, CSVRow& data);

} //namespace common
} //namespace ar


/* usage:
int main()
{
    std::vector<Pose2d> poseList;
    std::ifstream file("plop.csv");

    CSVRow row;
    while(file >> row)
    {
        Pose2d pose;
        pose.x = std::stof(row[0]);
        pose.y = std::stof(row[1]);
        pose.theta = std::stof(row[2]);
        poseList.push_back(pose);
    }
}
*/ 