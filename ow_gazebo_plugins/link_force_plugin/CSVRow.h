#ifndef CSVRow_h
#define CSVRow_h

// Adapted from https://stackoverflow.com/questions/1120140/how-can-i-read-and-parse-csv-files-in-c
#include <sstream>
#include <vector>
#include <string>

class CSVRow
{
public:
  std::string const& operator[](std::size_t index) const
  {
    return m_data[index];
  }
  std::size_t size() const
  {
    return m_data.size();
  }
  void readNextRow(std::istream& str)
  {
    std::string line;
    std::getline(str, line);

    std::stringstream lineStream(line);
    std::string cell;

    m_data.clear();
    while(std::getline(lineStream, cell, ','))
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

private:
  std::vector<std::string> m_data;
};

// Easy operator for piping a file or other stream to a CSVRow
std::istream& operator>>(std::istream& str, CSVRow& data)
{
  data.readNextRow(str);
  return str;
}

#endif
