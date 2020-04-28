/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


#ifdef ARILES_TRACE_ENABLE
#    include <iostream>
#    include <libgen.h>
#    include <typeinfo>
#    include <cxxabi.h>

namespace ariles
{
    namespace debug
    {
        class Tracer
        {
        public:
            std::string tabulation_;
            const std::string function_name_;
            const std::string file_;
            const int line_number_;

        protected:
            std::size_t getDepth(const bool increment = true)
            {
                static std::size_t depth = 0;
                if (true == increment)
                {
                    return (depth++);
                }
                else
                {
                    return (--depth);
                }
            }

            template <class t_First, class... t_Args>
            void outputFirst(t_First &&first, t_Args &&... args)
            {
                std::cout << first;
                outputFirst(std::forward<t_Args>(args)...);
            }

            template <class t_Last>
            void outputFirst(t_Last &&last)
            {
                std::cout << last << std::endl;
            }


        public:
            Tracer(const std::string &function_name, const std::string &file, const int line_number)
              : function_name_(function_name), file_(file), line_number_(line_number)
            {
                tabulation_.assign(getDepth(true), ' ');

                std::cout << tabulation_                                  //
                          << ">>> Entering function: " << function_name_  //
                          << " | File: " << file_                         //
                          << " | Line: " << line_number_ << std::endl;
            }

            ~Tracer()
            {
                std::cout << tabulation_                                 //
                          << "<<< Leaving function: " << function_name_  //
                          << " | File: " << file_                        //
                          << " | Line: " << line_number_ << std::endl;
                getDepth(false);
            }


            template <class... t_Args>
            void output(t_Args &&... args)
            {
                std::cout << tabulation_;
                outputFirst(std::forward<t_Args>(args)...);
            }

            std::string demangle(const char *name) const
            {
                std::size_t len = 0;
                int status = 0;

                char *demangled_name = abi::__cxa_demangle(name, NULL, &len, &status);

                if (demangled_name != NULL)
                {
                    const std::string result(demangled_name);
                    free(demangled_name);
                    return (result);
                }
                else
                {
                    return ("");
                }
            }
        };
    }  // namespace debug
}  // namespace ariles

#    define ARILES_TRACE_FUNCTION                                                                                      \
        char trace_path[] = __FILE__;                                                                                  \
        ariles::debug::Tracer tracer(__func__, basename(trace_path), __LINE__);
   // ariles::debug::Tracer tracer(__PRETTY_FUNCTION__, basename(trace_path), __LINE__);
#    define ARILES_TRACE_ENTRY(entry_name) tracer.output("Processing entry: ", entry_name);
#    define ARILES_TRACE_TYPE(entry) tracer.output("Type: ", tracer.demangle(typeid(entry).name()));
#else
#    define ARILES_TRACE_FUNCTION
#    define ARILES_TRACE_ENTRY(entry_name)
#    define ARILES_TRACE_TYPE(entry)
#endif
