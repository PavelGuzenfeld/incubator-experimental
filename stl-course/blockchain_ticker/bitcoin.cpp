#include <cpr/cpr.h>
#include <iomanip>
#include <iostream>
#include <json/json.h>
#include <string>

void printTable(const Json::Value &data)
{
    std::cout << std::left << std::setw(5) << "Currency"
              << std::setw(15) << "15m"
              << std::setw(15) << "Last"
              << std::setw(15) << "Buy"
              << std::setw(15) << "Sell"
              << std::setw(5) << "Symbol" << std::endl;

    for (const auto &currency : data.getMemberNames())
    {
        const auto &values = data[currency];
        std::cout << std::left << std::setw(5) << currency
                  << std::setw(15) << values["15m"].asDouble()
                  << std::setw(15) << values["last"].asDouble()
                  << std::setw(15) << values["buy"].asDouble()
                  << std::setw(15) << values["sell"].asDouble()
                  << std::setw(5) << values["symbol"].asString() << std::endl;
    }
}

int main()
{
    cpr::Response response = cpr::Get(cpr::Url{"https://blockchain.info/ticker"});

    if (response.status_code != 200)
    {
        std::cerr << "Error: Unable to fetch data, status code: " << response.status_code << std::endl;
        return 1;
    }

    Json::CharReaderBuilder readerBuilder;
    Json::Value data;
    std::string errs;

    std::istringstream s(response.text);
    if (!Json::parseFromStream(readerBuilder, s, &data, &errs))
    {
        std::cerr << "Error parsing JSON: " << errs << std::endl;
        return 1;
    }

    printTable(data);

    return 0;
}
