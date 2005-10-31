#
# --define 'kernel kernel-version'

# Build the madwifi-module on the last kernel-devel installed package.
# Need to implement it for smp kernel.
%{!?kernel: %{expand: %%define        kernel          %(rpm -q kernel-devel --qf %%{version}-%%{release}\\n | sort | tail -1)}}
#
%define       mykversion        %(echo %{kernel} | sed -e s/smp// -)
%define       mykrelver         %(echo %{mykversion} | tr -s '-' '_')

%if %(echo %{kernel} | grep -c smp)
      %{expand:%%define myksmp -smp}
%endif

Summary: A linux device driver for Atheros chipsets (ar5210, ar5211, ar5212).
Name: madwifi
Version: 0.1223.20051030
Release: 3
License: GPL2
Group: System Environment/Kernel
URL: http://sourceforge.net/projects/madwifi/
Source0: %{name}.%{version}.tar.bz2
BuildRoot: %{_tmppath}/%{name}-%{version}-%{release}-root
Requires: /sbin/depmod
Requires: %{name}-module >= %{version}
BuildRequires: /usr/bin/uudecode
BuildRequires: /sbin/depmod
#BuildRequires: kernel-devel
BuildRequires:  /lib/modules/%{mykversion}/build/Makefile


%description 
This package contains the Multiband Atheros Driver for WiFi, A linux
device driver for 802.11a/b/g universal NIC cards - either Cardbus,
PCI or MiniPCI - that use Atheros chipsets (ar5210, ar5211, ar5212).

%package module
Summary: A linux device driver for Atheros chipsets (ar5210, ar5211, ar5212).
Group: System Environment/Kernel
Requires: kernel = %{mykversion}
Release: 2_%{mykrelver}

%description module
This package contains the Multiband Atheros Driver for WiFi, A linux
device driver for 802.11a/b/g universal NIC cards - either Cardbus,
PCI or MiniPCI - that use Atheros chipsets (ar5210, ar5211, ar5212).


%prep 
%setup -q 
find . -name Makefile\* | xargs perl -pi -e's,/sbin/depmod,: /sbin/depmod,'

%build 
export KERNELRELEASE=%{mykversion}
export KERNELPATH=/lib/modules/%{mykversion}/build
export KERNELCONF=/lib/modules/%{mykversion}/build/.config
export MODULEPATH=/lib/modules/%{mykversion}/net
export COPTS="-I /lib/modules/%{mykversion}/build/include/ $COPTS"
# export ATH_RATE=ath_rate/onoe
make 
cd tools ; make all ; cd ..

%install 
export KERNELRELEASE=%{mykversion}
export KERNELPATH=/lib/modules/%{mykversion}/build
export KERNELCONF=/lib/modules/%{mykversion}/build/.config
export MODULEPATH=/lib/modules/%{mykversion}/net
rm -rf %{buildroot}
# export ATH_RATE=ath_rate/onoe
make info
make install DESTDIR=%{buildroot} KERNELPATH=/lib/modules/%{mykversion}/build
mkdir -p  %{buildroot}/usr/local/bin
cd tools ; make install DESTDIR=%{buildroot} KERNELPATH=/lib/modules/%{mykversion}/build BINDIR=/usr/local/bin  ; cd ..

%post  module
/sbin/depmod -ae %{mykversion}

%postun  module
/sbin/depmod -ae %{mykversion}

%clean 
rm -rf %{buildroot}

%files
%doc COPYRIGHT README
%attr(0755,root,root) /usr/local/bin/*
%attr(0644,root,root) /usr/local/man/*

%files module
%defattr(-,root,root,-)
/lib/modules/%{mykversion}/net/*.*o

#
%changelog
* Sun Oct 30 2005  Patrick Pichon <Patrick.Pichon@laposte.net>
- Add Man pages

* Mon Oct 24 2005  Lyonel Vincent <>
- Fix the --rebuild option.

* Mon Oct 3 2005  Lyonel Vincent <>
- Build by default on lastest kernel installed

* Wed Jul 20 2005 Patrick Pichon <Patrick.Pichon@laposte.net
- target particular kernel

* Mon Oct 27 2004 Patrick Pichon <Patrick.Pichon@laposte.net>
- Handle 2.4 kernel

* Mon Sep 21 2004 Lyonel Vincent <>
- change the naming of the kernel module part.

* Mon Aug 23 2004 Patrick Pichon <Patrick.Pichon@laposte.net>
- Update WPA has been merged into HEAD

* Mon Jul 27 2004 Patrick Pichon <Patrick.Pichon@laposte.net>
- Update to handle WPA branche.

* Tue Jul 08 2004 Patrick Pichon <Patrick.Pichon@laposte.net>
- Initial Release
