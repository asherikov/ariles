help:
	-@grep --color -Ev "(^	)|(^$$)" Makefile
	-@grep --color -Ev "(^	)|(^$$)" GNUmakefile
	-@grep --color -Ev "(^	)|(^$$)" make/Makefile*


# release
#----------------------------------------------

update_version:
	sed -i -e "s=\(project([ a-zA-Z0-9_-]* VERSION\) [0-9]*\.[0-9]*\.[0-9]*)=\1 ${VERSION})=" CMakeLists.txt
	#done automatically
	#sed -i -e "s=<version>[0-9]*\.[0-9]*\.[0-9]*</version>=<version>${VERSION}</version>=" package.xml

travis-trusty-apt-clean:
	sudo rm \
        /etc/apt/sources.list.d/cassandra.list \
        /etc/apt/sources.list.d/cassandra.list.save \
        /etc/apt/sources.list.d/heroku-toolbelt.list \
        /etc/apt/sources.list.d/heroku-toolbelt.list.save \
        /etc/apt/sources.list.d/chris-lea-redis-server.list \
        /etc/apt/sources.list.d/chris-lea-redis-server.list.save \
        /etc/apt/sources.list.d/mongodb-3.4.list \
        /etc/apt/sources.list.d/mongodb-3.4.list.save \
        /etc/apt/sources.list.d/computology_apt-backport.list \
        /etc/apt/sources.list.d/computology_apt-backport.list.save \
        /etc/apt/sources.list.d/openjdk-r-java-ppa.list \
        /etc/apt/sources.list.d/openjdk-r-java-ppa.list.save \
        /etc/apt/sources.list.d/couchdb.list \
        /etc/apt/sources.list.d/couchdb.list.save \
        /etc/apt/sources.list.d/pgdg.list \
        /etc/apt/sources.list.d/pgdg.list.save \
        /etc/apt/sources.list.d/pollinate.list \
        /etc/apt/sources.list.d/pollinate.list.save \
        /etc/apt/sources.list.d/github_git-lfs.list \
        /etc/apt/sources.list.d/github_git-lfs.list.save \
        /etc/apt/sources.list.d/rabbitmq_rabbitmq-server.list \
        /etc/apt/sources.list.d/rabbitmq_rabbitmq-server.list.save \
        /etc/apt/sources.list.d/git-ppa.list \
        /etc/apt/sources.list.d/git-ppa.list.save \
        /etc/apt/sources.list.d/webupd8team-java-ppa.list \
        /etc/apt/sources.list.d/webupd8team-java-ppa.list.save \
        /etc/apt/sources.list.d/google-chrome.list \
        /etc/apt/sources.list.d/google-chrome.list.save

