/**
    @file
    @author  Alexander Sherikov
    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)
*/


// Define constructors if requested
#ifdef ARILES_CONSTRUCTOR
    /**
     * Define constructors for the given class.
     */
    template <class t_Reader>
        ARILES_CONSTRUCTOR(
                t_Reader &reader,
                const std::string &node_name
                ARILES_CONFIGURABLE_PARAMETERS_ARG)
    {
        readConfig(reader, node_name, ARILES_CONFIGURABLE_PARAMETERS_ARG_VALUE);
    }

    template <class t_Reader>
        explicit ARILES_CONSTRUCTOR(
                t_Reader &reader
                ARILES_CONFIGURABLE_PARAMETERS_ARG_WITH_COMMA
                ARILES_IS_CHILD_ENABLER_TYPE(ariles::ReaderBase, t_Reader) * = NULL)
    {
        readConfig(reader, ARILES_CONFIGURABLE_PARAMETERS_ARG_VALUE);
    }
#endif



/**
 * @brief Read configuration (assuming the configuration node
 * to be in the root).
 *
 * @param[in] reader configuration reader
 */
template <class t_Reader>
    void readConfig(t_Reader            & reader
                    ARILES_CONFIGURABLE_PARAMETERS_ARG)
{
    ariles::reader::readEntry(reader, *this, this->getConfigSectionID(), ARILES_CONFIGURABLE_PARAMETERS_ARG_VALUE);
}


/**
 * @brief Read configuration (assuming the configuration node
 * to be in the root).
 *
 * @param[in] reader configuration reader
 * @param[in] node_name   node name, the default is used if empty
 */
template <class t_Reader>
    void readConfig(t_Reader            & reader,
                    const std::string   & node_name
                    ARILES_CONFIGURABLE_PARAMETERS_ARG)
{
    ariles::reader::readEntry(reader, *this, node_name, ARILES_CONFIGURABLE_PARAMETERS_ARG_VALUE);
}


/**
 * @brief Read configuration (assuming the configuration node
 * to be in the root).
 *
 * @param[in] reader configuration reader
 * @param[in] node_name   node name, the default is used if empty
 *
 * @note Intercept implicit conversion of a pointer to bool.
 */
template <class t_Reader>
    void readConfig(t_Reader            & reader,
                    const char          * node_name
                    ARILES_CONFIGURABLE_PARAMETERS_ARG)
{
    ariles::reader::readEntry(reader, *this, node_name, ARILES_CONFIGURABLE_PARAMETERS_ARG_VALUE);
}


/**
 * @brief Read configuration (assuming the configuration node
 * to be in the root).
 *
 * @param[in] file_name file name
 */
template <class t_Reader, class t_ReaderInitializer>
    void readConfig(t_ReaderInitializer         &reader_initializer
                    ARILES_CONFIGURABLE_PARAMETERS_ARG_WITH_COMMA
                    ARILES_IS_CHILD_ENABLER_TYPE(ariles::ReaderBase, t_Reader) * = NULL)
{
    t_Reader reader(reader_initializer);
    ariles::reader::readEntry(reader, *this, this->getConfigSectionID(), ARILES_CONFIGURABLE_PARAMETERS_ARG_VALUE);
}


/**
 * @brief Read configuration (assuming the configuration node
 * to be in the root).
 *
 * @param[in] file_name file name
 */
template <class t_Bridge, class t_ReaderInitializer>
    void readConfig(t_ReaderInitializer         &reader_initializer
                    ARILES_CONFIGURABLE_PARAMETERS_ARG_WITH_COMMA
                    ARILES_IS_CHILD_ENABLER_TYPE(ariles::BridgeSelectorBase, t_Bridge) * = NULL)
{
    readConfig<typename t_Bridge::Reader>(reader_initializer, ARILES_CONFIGURABLE_PARAMETERS_ARG_VALUE);
}


/**
 * @brief Read configuration (assuming the configuration node
 * to be in the root).
 *
 * @param[in] file_name file name
 * @param[in] node_name   node name, the default is used if empty
 */
template <class t_Reader, class t_ReaderInitializer>
    void readConfig(t_ReaderInitializer         &reader_initializer,
                    const std::string           &node_name
                    ARILES_CONFIGURABLE_PARAMETERS_ARG_WITH_COMMA
                    ARILES_IS_CHILD_ENABLER_TYPE(ariles::ReaderBase, t_Reader) * = NULL)
{
    t_Reader reader(reader_initializer);
    ariles::reader::readEntry(reader, *this, node_name, ARILES_CONFIGURABLE_PARAMETERS_ARG_VALUE);
}


/**
 * @brief Read configuration (assuming the configuration node
 * to be in the root).
 *
 * @param[in] file_name file name
 * @param[in] node_name   node name, the default is used if empty
 */
template <class t_Bridge, class t_ReaderInitializer>
    void readConfig(t_ReaderInitializer         &reader_initializer,
                    const std::string           &node_name
                    ARILES_CONFIGURABLE_PARAMETERS_ARG_WITH_COMMA
                    ARILES_IS_CHILD_ENABLER_TYPE(ariles::BridgeSelectorBase, t_Bridge) * = NULL)
{
    readConfig<typename t_Bridge::Reader>(reader_initializer, node_name, ARILES_CONFIGURABLE_PARAMETERS_ARG_VALUE);
}


/**
 * @brief Read configuration (assuming the configuration node
 * to be in the root).
 *
 * @param[in] file_name file name
 * @param[in] node_name   node name, the default is used if empty
 *
 * @note Intercept implicit conversion of a pointer to bool.
 */
template <class t_Reader, class t_ReaderInitializer>
    void readConfig(t_ReaderInitializer         &reader_initializer,
                    const char                  *node_name
                    ARILES_CONFIGURABLE_PARAMETERS_ARG_WITH_COMMA
                    ARILES_IS_CHILD_ENABLER_TYPE(ariles::ReaderBase, t_Reader) * = NULL)
{
    t_Reader reader(reader_initializer);
    ariles::reader::readEntry(reader, *this, node_name, ARILES_CONFIGURABLE_PARAMETERS_ARG_VALUE);
}


/**
 * @brief Read configuration (assuming the configuration node
 * to be in the root).
 *
 * @param[in] file_name file name
 * @param[in] node_name   node name, the default is used if empty
 *
 * @note Intercept implicit conversion of a pointer to bool.
 */
template <class t_Bridge, class t_ReaderInitializer>
    void readConfig(t_ReaderInitializer         &reader_initializer,
                    const char                  *node_name
                    ARILES_CONFIGURABLE_PARAMETERS_ARG_WITH_COMMA
                    ARILES_IS_CHILD_ENABLER_TYPE(ariles::BridgeSelectorBase, t_Bridge) * = NULL)
{
    readConfig<typename t_Bridge::Reader>(reader_initializer, node_name, ARILES_CONFIGURABLE_PARAMETERS_ARG_VALUE);
}


// ============================================


/**
 * @brief Write configuration
 *
 * @param[in,out] writer configuration writer
 */
template <class t_Writer>
    void writeConfig(   t_Writer& writer
                        ARILES_CONFIGURABLE_PARAMETERS_ARG) const
{
    writeConfig(writer, this->getConfigSectionID(), ARILES_CONFIGURABLE_PARAMETERS_ARG_VALUE);
}


/**
 * @brief Write configuration
 *
 * @param[in,out] writer configuration writer
 * @param[in] node_name   node name, the default is used if empty
 */
template <class t_Writer>
    void writeConfig(t_Writer& writer,
                     const std::string &node_name
                     ARILES_CONFIGURABLE_PARAMETERS_ARG) const
{
    writer.initRoot();
    ariles::writer::writeEntry(writer, *this, node_name, ARILES_CONFIGURABLE_PARAMETERS_ARG_VALUE);
    writer.flush();
}


/**
 * @brief Write configuration
 *
 * @param[in,out] writer configuration writer
 * @param[in] node_name   node name, the default is used if empty
 */
template <class t_Writer>
    void writeConfig(t_Writer& writer,
                     const char *node_name
                     ARILES_CONFIGURABLE_PARAMETERS_ARG) const
{
    writer.initRoot();
    ariles::writer::writeEntry(writer, *this, node_name, ARILES_CONFIGURABLE_PARAMETERS_ARG_VALUE);
    writer.flush();
}


/**
 * @brief Write configuration.
 *
 * @param[in] file_name file name
 */
template <class t_Writer, class t_WriterInitializer>
    void writeConfig(   t_WriterInitializer &writer_initializer
                        ARILES_CONFIGURABLE_PARAMETERS_ARG_WITH_COMMA
                        ARILES_IS_CHILD_ENABLER_TYPE(ariles::WriterBase, t_Writer) * = NULL) const
{
    t_Writer writer(writer_initializer);
    writeConfig(writer, ARILES_CONFIGURABLE_PARAMETERS_ARG_VALUE);
}


/**
 * @brief Write configuration.
 *
 * @param[in] file_name file name
 */
template <class t_Bridge, class t_WriterInitializer>
    void writeConfig(   t_WriterInitializer &writer_initializer
                        ARILES_CONFIGURABLE_PARAMETERS_ARG_WITH_COMMA
                        ARILES_IS_CHILD_ENABLER_TYPE(ariles::BridgeSelectorBase, t_Bridge) * = NULL) const
{
    writeConfig<typename t_Bridge::Writer>(writer_initializer, ARILES_CONFIGURABLE_PARAMETERS_ARG_VALUE);
}


/**
 * @brief Write configuration.
 *
 * @param[in] file_name file name
 * @param[in] node_name node name, the default is used if empty
 */
template <class t_Writer, class t_WriterInitializer>
    void writeConfig(   t_WriterInitializer &writer_initializer,
                        const std::string &node_name
                        ARILES_CONFIGURABLE_PARAMETERS_ARG_WITH_COMMA
                        ARILES_IS_CHILD_ENABLER_TYPE(ariles::WriterBase, t_Writer) * = NULL) const
{
    t_Writer writer(writer_initializer);
    writeConfig(writer, node_name, ARILES_CONFIGURABLE_PARAMETERS_ARG_VALUE);
}


/**
 * @brief Write configuration.
 *
 * @param[in] file_name file name
 * @param[in] node_name node name, the default is used if empty
 */
template <class t_Writer, class t_WriterInitializer>
    void writeConfig(   t_WriterInitializer &writer_initializer,
                        const char *node_name
                        ARILES_CONFIGURABLE_PARAMETERS_ARG_WITH_COMMA
                        ARILES_IS_CHILD_ENABLER_TYPE(ariles::WriterBase, t_Writer) * = NULL) const
{
    t_Writer writer(writer_initializer);
    writeConfig(writer, node_name, ARILES_CONFIGURABLE_PARAMETERS_ARG_VALUE);
}


/**
 * @brief Write configuration.
 *
 * @param[in] file_name file name
 * @param[in] node_name node name, the default is used if empty
 */
template <class t_Bridge, class t_WriterInitializer>
    void writeConfig(   t_WriterInitializer &writer_initializer,
                        const std::string &node_name
                        ARILES_CONFIGURABLE_PARAMETERS_ARG_WITH_COMMA
                        ARILES_IS_CHILD_ENABLER_TYPE(ariles::BridgeSelectorBase, t_Bridge) * = NULL) const
{
    writeConfig<typename t_Bridge::Writer>(writer_initializer, node_name, ARILES_CONFIGURABLE_PARAMETERS_ARG_VALUE);
}


/**
 * @brief Write configuration.
 *
 * @param[in] file_name file name
 * @param[in] node_name node name, the default is used if empty
 */
template <class t_Bridge, class t_WriterInitializer>
    void writeConfig(   t_WriterInitializer &writer_initializer,
                        const char *node_name
                        ARILES_CONFIGURABLE_PARAMETERS_ARG_WITH_COMMA
                        ARILES_IS_CHILD_ENABLER_TYPE(ariles::BridgeSelectorBase, t_Bridge) * = NULL) const
{
    writeConfig<typename t_Bridge::Writer>(writer_initializer, node_name, ARILES_CONFIGURABLE_PARAMETERS_ARG_VALUE);
}

