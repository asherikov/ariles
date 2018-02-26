/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


class FixtureBase
{
    public:
        FixtureBase()
        {
        }

        const std::string getInitializer(const std::string & string_id)
        {
            return (string_id);
        }
};
