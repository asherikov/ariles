# $FreeBSD$

PORTNAME=		ariles
DISTVERSION=	2.0.0
CATEGORIES=		devel

MAINTAINER=		asherikov@yandex.com
COMMENT=		C++ serialization / configuration library

LICENSE=		APACHE20

BUILD_DEPENDS=cmake:devel/cmake ${ARILES_DEPENDS}
BUILD_DEPENDS=${ARILES_DEPENDS}
RUN_DEPENDS=${ARILES_DEPENDS}

USES=			cmake

USE_GITHUB=		yes
GH_ACCOUNT=		asherikov
GH_PROJECT=		ariles
GH_TAGNAME=		2.0.0

USE_LDCONFIG=	yes

ARILES_DEPENDS=	${LOCALBASE}/include/eigen3/Eigen/Core:math/eigen3 \
				${LOCALBASE}/include/boost/lexical_cast.hpp:devel/boost-libs

CMAKE_ARGS+=	-DARILES_VISITORS_DEFAULT_MODE=OFF \
				-DARILES_BUILD_REGRESSION_TESTS=OFF

OPTIONS_SUB=	yes
OPTIONS_DEFINE=	MSGPACK RAPIDJSON PUGIXML OCTAVE JSONNET GRAPHVIZ NAMEVALUE YAMLCPP

JSONNET_IMPLIES=RAPIDJSON

MSGPACK_DESC=	msgpack support
YAMLCPP_DESC=	yaml-cpp support
RAPIDJSON_DESC=	RapisJSON support
PUGIXML_DESC=	PugiXML support
OCTAVE_DESC=	Octave script support (output only, no dependencies)
JSONNET_DESC=	Jsonnet preprocessing for JSON
GRAPHVIZ_DESC=	dot file output support
NAMEVALUE_DESC=	name-value pair outout support

MSGPACK_LIB_DEPENDS=	libmsgpackc.so:devel/msgpack
YAMLCPP_LIB_DEPENDS=	libyaml-cpp.so:devel/yaml-cpp
RAPIDJSON_BUILD_DEPENDS=${LOCALBASE}/include/rapidjson/document.h:devel/rapidjson
PUGIXML_LIB_DEPENDS=	libpugixml.so:textproc/pugixml
JSONNET_LIB_DEPENDS=	libjsonnet.so:devel/jsonnet

MSGPACK_CMAKE_ON=	-DARILES_VISITOR_msgpack=ON
YAMLCPP_CMAKE_ON=	-DARILES_VISITOR_yaml_cpp=ON
RAPIDJSON_CMAKE_ON=	-DARILES_VISITOR_rapidjson=ON
PUGIXML_CMAKE_ON=	-DARILES_VISITOR_pugixml=ON
JSONNET_CMAKE_ON=	-DARILES_VISITOR_jsonnet=ON
OCTAVE_CMAKE_ON=	-DARILES_VISITOR_octave=ON
GRAPHVIZ_CMAKE_ON=	-DARILES_VISITOR_graphviz=ON
NAMEVALUE_CMAKE_ON=	-DARILES_VISITOR_namevalue=ON

.include <bsd.port.mk>
