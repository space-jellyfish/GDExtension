#ifndef MAP_PARSER_H
#define MAP_PARSER_H

#include <unordered_map>


class MapParser {
public:
    
private:
    std::unordered_map<std::string, std::string> contents;
}
void 
warthog::gm_parser::parse_header(std::fstream& mapfs)
{
	// read header fields
	std::unordered_map<std::string, std::string> contents;
	for(int i=0; i < 3; i++)
	{
		std::string hfield, hvalue;
		mapfs >> hfield;
		if(mapfs.good())
		{
			mapfs >> hvalue;
			if(mapfs.good())
			{
				contents[hfield] = hvalue;
			}
			else
			{
				std::cerr << "err; map load failed. could not read header." << 
					hfield << std::endl;
				exit(1);
			}
		}
		else
		{
			std::cerr << "err;  map load failed. format looks wrong."<<std::endl;
			exit(1);
		}
	}

	this->header_.type_ = contents[std::string("type")];
	if(this->header_.type_.compare("octile") != 0)
	{
		std::cerr << "err; map type " << this->header_.type_ << 
			"is unknown. known types: octile "<<std::endl;;
		exit(1);
	}

	this->header_.height_ = (uint32_t)atoi(contents[std::string("height")].c_str());
	if(this->header_.height_ == 0)
	{
		std::cerr << "err; map file specifies invalid height. " << std::endl;
		exit(1);
	}

	this->header_.width_ = (uint32_t)atoi(contents[std::string("width")].c_str());
	if(this->header_.width_ == 0)
	{
		std::cerr << "err; map file specifies invalid width. " << std::endl;
		exit(1);
	}

}

#endif