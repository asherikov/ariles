/**
    @file
    @author  Alexander Sherikov

    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "internal/helpers.h"
#include "internal/reader_base.h"
#include "internal/writer_base.h"

// These defines are always necessary
#define ARILES_TYPED_ENTRY_(entry, type) ARILES_TYPED_NAMED_ENTRY(type, entry##_, #entry)
#define ARILES_TYPED_ENTRY(entry, type)  ARILES_TYPED_NAMED_ENTRY(type, entry, #entry)


#ifndef ARILES_DISABLE
#   define ARILES_ENABLED


    #define ARILES_ENTRY_(entry)     ARILES_NAMED_ENTRY(entry##_, #entry)
    #define ARILES_ENTRY(entry)      ARILES_NAMED_ENTRY(entry, #entry)


    #define ARILES_WRITE_NAMED_ENTRY(entry, name)    ariles::writer::writeEntry(writer, entry, name, param);

    #define ARILES_WRITE_ENTRY_(entry)   ARILES_WRITE_NAMED_ENTRY(entry##_, #entry)
    #define ARILES_WRITE_ENTRY(entry)    ARILES_WRITE_NAMED_ENTRY(entry, #entry)

    #define ARILES_WRITE_PARENT(parent_class)  parent_class::writeConfigEntries(writer, param);


    #define ARILES_READ_NAMED_ENTRY(entry, name)  ariles::reader::readEntry(reader, entry, name, param);

    #define ARILES_READ_ENTRY_(entry)    ARILES_READ_NAMED_ENTRY(entry##_, #entry);
    #define ARILES_READ_ENTRY(entry)     ARILES_READ_NAMED_ENTRY(entry, #entry);

    #define ARILES_READ_PARENT(parent_class)  parent_class::readConfigEntries(reader, param);

    // ----------------------------

    namespace ariles
    {
        /**
         * @brief Configurable base class.
         */
        class ARILES_VISIBILITY_ATTRIBUTE CommonConfigurableBase
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~CommonConfigurableBase() {}
                CommonConfigurableBase() {}


                virtual const ConfigurableParameters &getArilesConfigurableParameters() const = 0;


            public:
                /**
                 * @brief Return the default name of a configuration node
                 * corresponding to this class
                 *
                 * @return the name
                 *
                 * @attention Implementation of this method is added
                 * automatically upon inclusion of define_accessors.h if
                 * ARILES_SECTION_ID is defined.
                 */
                virtual const std::string & getConfigSectionID() const = 0;


                /**
                 * @brief This function is called automaticaly after reading
                 * a configuration file. Does nothing by default.
                 */
                virtual void finalize() {};


                /**
                 * @brief Set members to their default values.
                 */
                virtual void setDefaults() = 0;


                /**
                 * @brief Get number of entries in the corresponding
                 * configuration node.
                 *
                 * @return number of entries
                 */
                virtual std::size_t getNumberOfEntries() const = 0;


                /// @{
                /**
                 * These functions are always defined automatically.
                 */
                virtual void writeConfigEntries(ariles::WriterBase & writer,
                                                const ConfigurableParameters & param) const = 0;

                virtual void readConfigEntries( ariles::ReaderBase & reader,
                                                const ConfigurableParameters & param) = 0;
                /// @}
        };


        class ARILES_VISIBILITY_ATTRIBUTE StrictConfigurableBase : public ariles::CommonConfigurableBase
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~StrictConfigurableBase() {}
                StrictConfigurableBase() {}


                virtual const ConfigurableParameters &getArilesConfigurableParameters() const
                {
                    static ConfigurableParameters parameters(true);
                    return (parameters);
                }
        };


        class ARILES_VISIBILITY_ATTRIBUTE RelaxedConfigurableBase : public ariles::CommonConfigurableBase
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~RelaxedConfigurableBase() {}
                RelaxedConfigurableBase() {}


                virtual const ConfigurableParameters &getArilesConfigurableParameters() const
                {
                    static ConfigurableParameters parameters(false);
                    return (parameters);
                }
        };



        /// Default configurable base is strict
        class ARILES_VISIBILITY_ATTRIBUTE ConfigurableBase : public ariles::StrictConfigurableBase
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~ConfigurableBase() {}
                ConfigurableBase() {}
        };
    }

#   include "adapters/basic.h"
#   include "adapters/included.h"

#else

#   define ARILES_DISABLED

    namespace ariles
    {
        // Some classes may inherit from this
        class ARILES_VISIBILITY_ATTRIBUTE StrictConfigurableBase
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the
                 * child classes through a base pointer.
                 */
                ~StrictConfigurableBase() {}
        };


        // Some classes may inherit from this
        class ARILES_VISIBILITY_ATTRIBUTE RelaxedConfigurableBase
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the
                 * child classes through a base pointer.
                 */
                ~RelaxedConfigurableBase() {}
        };


        // Some classes may inherit from this
        class ARILES_VISIBILITY_ATTRIBUTE ConfigurableBase : ariles::StrictConfigurableBase
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the
                 * child classes through a base pointer.
                 */
                ~ConfigurableBase() {}
        };
    }

#endif
