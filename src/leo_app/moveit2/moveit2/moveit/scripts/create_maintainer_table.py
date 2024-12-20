#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2024 AUBO Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PickNik Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


from __future__ import print_function

import os
import sys
import webbrowser

from catkin_pkg.packages import find_packages

maintainers_dict = {
    "Ioan Sucan": "isucan",
    "Michael Ferguson": "mikeferguson",
    "Sachin Chitta": "sachinchitta",
    "G.A. vd. Hoorn": "gavanderhoorn",
    "Dave Coleman": "davetcoleman",
    "Acorn Pooley": "acorn",
    "Jon Binney": "jonbinney",
    "Matei Ciocarlie": "mateiciocarlie",
    "Michael Görner".decode("utf8"): "v4hn",
    "Robert Haschke": "rhaschke",
    "Ian McMahon": "IanTheEngineer",
    "Isaac I. Y. Saito": "130s",
    "Mathias Lüdtke".decode("utf8"): "ipa-mdl",
    "Ryan Luna": "ryanluna",
    "Chittaranjan Srinivas Swaminathan": "ksatyaki",
    "Chittaranjan S Srinivas": "ksatyaki",
}


def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)


def author_to_github(maintainer):
    try:
        name = maintainers_dict[maintainer.name]
    except KeyError:
        eprint("Missing maintainer: ", maintainer.name)
        name = maintainer.email
    return name


def template_file(src, dst, subs):
    print("++ Templating '{0}'".format(src))
    with open(src, "r") as f:
        data = f.read()
    for k, v in subs.items():
        data = data.replace(k, v)
    with open(dst, "w+") as f:
        f.write(data)
    print("++ Webpage ready at '{0}'".format(dst))
    webbrowser.open(dst)


def create_travis_badge(package_name):
    output = ""
    # Indigo
    output += "<td>"
    output += (
        "<a href='http://build.ros.org/view/Isrc_uT/job/Isrc_uT__"
        + package_name
        + "__ubuntu_trusty__source/'>"
        "<img src='http://build.ros.org/buildStatus/icon?job=Isrc_uT__"
        + package_name
        + "__ubuntu_trusty__source'></a>"
    )
    output += "</td><td>"
    output += (
        "<a href='http://build.ros.org/view/Ibin_uT64/job/Ibin_uT64__"
        + package_name
        + "__ubuntu_trusty_amd64__binary/'>"
        "<img src='http://build.ros.org/buildStatus/icon?job=Ibin_uT64__"
        + package_name
        + "__ubuntu_trusty_amd64__binary'></a>"
    )
    output += "</td>"

    # Jade
    output += "<td>"
    output += (
        "<a href='http://build.ros.org/view/Jsrc_uT/job/Jsrc_uT__"
        + package_name
        + "__ubuntu_trusty__source/'>"
        "<img src='http://build.ros.org/buildStatus/icon?job=Jsrc_uT__"
        + package_name
        + "__ubuntu_trusty__source'></a>"
    )
    output += "</td><td>"
    output += (
        "<a href='http://build.ros.org/view/Jbin_uT64/job/Jbin_uT64__"
        + package_name
        + "__ubuntu_trusty_amd64__binary/'>"
        "<img src='http://build.ros.org/buildStatus/icon?job=Jbin_uT64__"
        + package_name
        + "__ubuntu_trusty_amd64__binary'></a>"
    )
    output += "</td>"

    # Kinetic
    output += "<td>"
    output += (
        "<a href='http://build.ros.org/view/Ksrc_uX/job/Ksrc_uX__"
        + package_name
        + "__ubuntu_xenial__source/'>"
        "<img src='http://build.ros.org/buildStatus/icon?job=Ksrc_uX__"
        + package_name
        + "__ubuntu_xenial__source'></a>"
    )
    output += "</td><td>"
    output += (
        "<a href='http://build.ros.org/view/Kbin_uX64/job/Kbin_uX64__"
        + package_name
        + "__ubuntu_xenial_amd64__binary/'>"
        "<img src='http://build.ros.org/buildStatus/icon?job=Kbin_uX64__"
        + package_name
        + "__ubuntu_xenial_amd64__binary'></a>"
    )
    output += "</td>"
    return output


def get_first_folder(path):
    head, tail = os.path.split(path)
    components = []
    while len(tail) > 0:
        components.insert(0, tail)
        head, tail = os.path.split(head)
    return components[0]


def populate_package_data(path, package):
    output = (
        "<td><a href='https://github.com/ros-planning/"
        + get_first_folder(path)
        + "'>"
        + package.name
        + "</a></td>"
    )
    output += "<td>" + package.version + "</td>"
    output += "<td>"
    first = True
    for maintainer in package.maintainers:
        author = author_to_github(maintainer)
        if first:
            first = False
        else:
            output += ", "
        output += "<a href='https://github.com/" + author + "'>" + author + "</a>"
    output += "</td>"
    output += create_travis_badge(package.name)
    return output


def list_moveit_packages():
    """Create list of MoveIt packages."""
    output = ""
    packages = find_packages(os.getcwd())

    for path, package in packages.items():
        output += "<tr>"
        output += populate_package_data(path, package)
        output += "</tr>"

    # Save to file
    basepath = os.path.dirname(os.path.realpath(__file__))
    template_file(
        os.path.join(basepath, "maintainer_table_template.html"),
        os.path.join(basepath, "index.html"),
        {"CONTENTS": output},
    )


if __name__ == "__main__":
    sys.exit(list_moveit_packages())
