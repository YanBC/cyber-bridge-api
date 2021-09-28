for pf in `find cyber modules -iname *.proto`
do 
    # echo $pf
    protoc -I . --python_out . $pf
done
