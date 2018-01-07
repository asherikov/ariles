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

// These defines are always necessary
#define ARILES_TYPED_ENTRY_(entry, type) ARILES_TYPED_NAMED_ENTRY(type, entry##_, #entry)
#define ARILES_TYPED_ENTRY(entry, type)  ARILES_TYPED_NAMED_ENTRY(type, entry, #entry)


#ifdef ARILES_NAMESPACE_0
#   define ARILES_ENABLED


#   include "internal/process_namespaces.h"


    #define ARILES_ENTRY_(entry)     ARILES_NAMED_ENTRY(entry##_, #entry)
    #define ARILES_ENTRY(entry)      ARILES_NAMED_ENTRY(entry, #entry)


    #define ARILES_WRITE_NAMED_ENTRY(entry, name)    ariles::writer::writeEntry(writer, entry, name);

    #define ARILES_WRITE_ENTRY_(entry)   ARILES_WRITE_NAMED_ENTRY(entry##_, #entry)
    #define ARILES_WRITE_ENTRY(entry)    ARILES_WRITE_NAMED_ENTRY(entry, #entry)

    #define ARILES_WRITE_PARENT(parent_class)  parent_class::writeConfigEntries(writer);


    #define ARILES_READ_NAMED_ENTRY(entry, name)  ariles::reader::readEntry(reader, entry, name, crash_on_missing_entry);

    #define ARILES_READ_ENTRY_(entry)    ARILES_READ_NAMED_ENTRY(entry##_, #entry);
    #define ARILES_READ_ENTRY(entry)     ARILES_READ_NAMED_ENTRY(entry, #entry);

    #define ARILES_READ_PARENT(parent_class)  parent_class::readConfigEntries(reader, crash_on_missing_entry);

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


                virtual bool getCrashOnMissingEntryFlag() = 0;


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
                #define ARILES_NAMESPACE(config_namespace) \
                    virtual void writeConfigEntries(ariles::config_namespace::Writer &) const = 0; \
                    virtual void readConfigEntries( ariles::config_namespace::Reader & reader, \
                                                    const bool crash_flag) = 0;
                ARILES_MACRO_SUBSTITUTE(ARILES_NAMESPACE_LIST)
                #undef ARILES_NAMESPACE
                /// @}
        };


        class ARILES_VISIBILITY_ATTRIBUTE StrictConfigurableBase : public ariles::CommonConfigurableBase
        {
            protected:
                bool getCrashOnMissingEntryFlag() {return (true);}


            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~StrictConfigurableBase() {}
                StrictConfigurableBase() {}
        };


        class ARILES_VISIBILITY_ATTRIBUTE RelaxedConfigurableBase : public ariles::CommonConfigurableBase
        {
            protected:
                bool getCrashOnMissingEntryFlag() {return (false);}


            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~RelaxedConfigurableBase() {}
                RelaxedConfigurableBase() {}
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
#   include "adapters/selected.h"

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

#endif //ARILES_NAMESPACE_0
