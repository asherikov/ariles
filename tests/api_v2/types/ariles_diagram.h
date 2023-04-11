/**
    @file
    @author  Alexander Sherikov

    @copyright 2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


namespace ariles_tests
{
    class ArilesDependencyParameters : public ariles2::graphviz::Parameters
    {
    public:
        std::string id_;
        std::string url_;


    public:
        ArilesDependencyParameters(const std::string &id, const std::string &url) : id_(id), url_(url)
        {
        }

        virtual ariles2::graphviz::Parameters::NodeOptions getArilesNodeOptions(
                const std::string & /*id*/,
                const std::string & /*label*/) const
        {
            return (ariles2::graphviz::Parameters::NodeOptions(id_, url_, std::string("URL=\"") + url_ + "\""));
        }
    };


    class ArilesVisitorParameters : public ariles2::graphviz::Parameters
    {
    public:
        virtual ariles2::graphviz::Parameters::NodeOptions getArilesNodeOptions(
                const std::string &id,
                const std::string &label) const
        {
            std::string url_name = label;
            std::size_t pos = url_name.find("_");

            while (pos != std::string::npos)
            {
                url_name.replace(pos, 1, "__");
                pos = url_name.find("_", pos + 2);
            }

            return (ariles2::graphviz::Parameters::NodeOptions(
                    id,
                    label,
                    std::string("URL=\"https://asherikov.github.io/ariles/2/group__") + url_name + ".html\""));
        }
    };


    class ArilesDependency : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)
#include ARILES2_INITIALIZE

    public:
        ArilesDependencyParameters parameters_;

    public:
        ArilesDependency() : parameters_("", "")
        {
        }

        ArilesDependency(const std::string &id, const std::string &url) : parameters_(id, url)
        {
        }

        virtual ~ArilesDependency()
        {
        }

        const ariles2::graphviz::Parameters &arilesGetParameters(
                const ariles2::graphviz::Visitor & /*visitor*/) const override
        {
            return (parameters_);
        }
    };


    class ArilesVisitor : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v) ARILES2_ENTRY_(v, dependencies)
#include ARILES2_INITIALIZE

    public:
        std::vector<ArilesDependency> dependencies_;

    public:
        virtual ~ArilesVisitor()
        {
        }

        const ariles2::graphviz::Parameters &arilesGetParameters(
                const ariles2::graphviz::Visitor & /*visitor*/) const override
        {
            static const ArilesVisitorParameters parameters;
            return (parameters);
        }
    };


    class ArilesDiagram : public ariles2::DefaultBase
    {
#define ARILES2_DEFAULT_ID "ariles"
#define ARILES2_ENTRIES(v) ARILES2_ENTRY_(v, visitors)
#include ARILES2_INITIALIZE

    public:
        std::map<std::string, ArilesVisitor> visitors_;


    public:
        ArilesDiagram()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }


        void arilesVisit(const ariles2::Defaults & /*visitor*/, const ariles2::Defaults::Parameters & /*param*/)
        {
            visitors_["namevalue"] = ArilesVisitor();
            visitors_["graphviz"] = ArilesVisitor();
            visitors_["octave"] = ArilesVisitor();

            visitors_["jsonnet"] = ArilesVisitor();
            visitors_["jsonnet"].dependencies_.push_back(ArilesDependency("jsonnet", "https://jsonnet.org/"));

            visitors_["msgpack"] = ArilesVisitor();
            visitors_["msgpack"].dependencies_.push_back(ArilesDependency("msgpack", "https://msgpack.org/"));

            visitors_["pugixml"] = ArilesVisitor();
            visitors_["pugixml"].dependencies_.push_back(ArilesDependency("pugixml", "https://pugixml.org/"));

            visitors_["rapidjson"] = ArilesVisitor();
            visitors_["rapidjson"].dependencies_.push_back(ArilesDependency("rapidjson", "https://rapidjson.org/"));

            visitors_["rosparam"] = ArilesVisitor();
            visitors_["rosparam"].dependencies_.push_back(ArilesDependency("ros", "https://www.ros.org/"));

            visitors_["yaml_cpp"] = ArilesVisitor();
            visitors_["yaml_cpp"].dependencies_.push_back(
                    ArilesDependency("yaml_cpp", "https://github.com/jbeder/yaml-cpp"));
        }

        void randomize()
        {
        }
    };
}  // namespace ariles_tests
