#include "common/CSVRow.h"

#include <algorithm>

namespace ar{
namespace common{

CSVRow::CSVRow()
    : CSVRow(';')
{}

CSVRow::CSVRow(char seperator)
    : m_seperator(seperator)
{}

int CSVRow::find(const std::string& string, const std::string& prefix, const std::string& suffix)
{
	return find(prefix + string + suffix);
}

int CSVRow::find(const std::string& string)
{
    auto begin = m_data.begin();
    auto end = m_data.end();
    auto it = std::find(begin, end, string);
    if(it == end)
    {
        return -1;
    }
    else
    {
        return static_cast<int>(it - begin);
    }
}

std::string const& CSVRow::operator[](std::size_t index) const
{
    return m_data[index];
}
std::size_t CSVRow::size() const
{
    return m_data.size();
}

void CSVRow::readNextRow(std::istream& str)
{
    std::string line;
    std::getline(str, line);

    std::stringstream lineStream(line);
    std::string cell;

    m_data.clear();
    while(std::getline(lineStream, cell, m_seperator))
    {
        m_data.push_back(cell);
    }
    // This checks for a trailing comma with no data after it.
    if (!lineStream && cell.empty())
    {
        // If there was a trailing comma then add an empty element.
        m_data.push_back("");
    }
}

std::istream& operator>>(std::istream& str, CSVRow& data)
{
    data.readNextRow(str);
    return str;
}


} //namespace common
} //namespace ar
